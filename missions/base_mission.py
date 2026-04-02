#!/usr/bin/env python3
"""
Base class for all autonomous missions.

Every mission follows the same lifecycle:
    preflight check -> pre_execute -> execute -> post_execute -> save result

Subclasses implement execute() with their specific mission logic.
Safety monitors run throughout the entire lifecycle.
"""

import asyncio
import json
import logging
import time
from abc import ABC, abstractmethod
from pathlib import Path

logger = logging.getLogger(__name__)


class MissionResult:
    """Stores the outcome of a mission run."""

    def __init__(self, mission_name):
        self.mission_name = mission_name
        self.success = False
        self.start_time = None
        self.end_time = None
        self.elapsed_seconds = None
        self.aborted = False
        self.abort_reason = None
        self.data = {}  # mission-specific results (checkpoints, detections, etc.)

    def to_dict(self):
        return {
            "mission": self.mission_name,
            "success": self.success,
            "elapsed_seconds": self.elapsed_seconds,
            "aborted": self.aborted,
            "abort_reason": self.abort_reason,
            "data": self.data,
        }


class BaseMission(ABC):
    """Abstract base for all autonomous missions.

    Safety contract:
        - run() checks pre_flight_check() before anything flies
        - Background monitors watch battery and flight mode throughout
        - execute() implementations must check is_safe_to_continue() in their loops
        - Any unhandled exception triggers emergency_abort()
    """

    def __init__(self, controller, config_path):
        """
        Args:
            controller: DroneController instance (already connected)
            config_path: path to JSON config file for this mission
        """
        self.controller = controller
        self.config = self._load_config(config_path)
        self.result = MissionResult(self.name)

    @property
    @abstractmethod
    def name(self) -> str:
        """Human-readable mission name."""
        ...

    @abstractmethod
    async def execute(self):
        """Core mission logic. Must check controller.is_safe_to_continue() in loops."""
        ...

    def _load_config(self, config_path):
        """Load and validate JSON config."""
        path = Path(config_path)
        if not path.exists():
            raise FileNotFoundError(f"Mission config not found: {config_path}")
        with open(path, "r") as f:
            config = json.load(f)
        self._validate_config(config)
        return config

    def _validate_config(self, config):
        """Override in subclass to validate required config fields."""
        pass

    async def pre_execute(self):
        """Setup before mission: arm, takeoff, start monitors. Override to customize."""
        altitude = self.config.get("altitude_m", 3.0)
        await self.controller.arm_and_takeoff(altitude)
        self.controller.start_monitors()

    async def post_execute(self):
        """Cleanup after mission: land, stop monitors."""
        await self.controller.land()
        await asyncio.sleep(10)
        self.controller.stop_monitors()

    async def run(self):
        """Full mission lifecycle. Returns MissionResult."""
        logger.info(f"\n{'=' * 60}")
        logger.info(f"MISSION: {self.name}")
        logger.info(f"{'=' * 60}\n")

        # Pre-flight safety gate
        if not await self.controller.pre_flight_check():
            logger.error("Pre-flight check FAILED. Mission aborted.")
            self.result.aborted = True
            self.result.abort_reason = "pre_flight_check_failed"
            return self.result

        try:
            await self.pre_execute()

            if not self.controller.is_safe_to_continue():
                raise RuntimeError("Safety check failed before execute")

            self.result.start_time = time.time()
            await self.execute()
            self.result.end_time = time.time()
            self.result.elapsed_seconds = self.result.end_time - self.result.start_time
            self.result.success = True

        except Exception as e:
            logger.error(f"Mission failed: {e}")
            self.result.aborted = True
            self.result.abort_reason = str(e)
            await self.controller.emergency_abort()

        finally:
            if not self.result.aborted:
                await self.post_execute()
            self.controller.stop_monitors()
            self._save_result()

        return self.result

    def _save_result(self):
        """Write mission result JSON to logs/ directory."""
        log_dir = Path(__file__).resolve().parent.parent / "logs"
        log_dir.mkdir(exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        log_path = log_dir / f"{self.name}_{timestamp}.json"
        with open(log_path, "w") as f:
            json.dump(self.result.to_dict(), f, indent=2)
        logger.info(f"Mission result saved to {log_path}")
