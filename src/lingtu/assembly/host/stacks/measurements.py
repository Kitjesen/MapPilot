"""Optional typed measurement readers; ProductControl retains process ownership."""

from __future__ import annotations

import os

from runtime.host import HostBuilder
from runtime.plugin_resolution import stack_module


def measurements(**config) -> HostBuilder:
    builder = HostBuilder()
    if not config.get("enable_measurements", False):
        return builder
    endpoint = stack_module(
        "measurements", "native_dds", seed_group="camera_sim",
        fallback="drivers.sim.measurement.dds_module.DdsMeasurementModule",
    )
    # Compilation constructs entries only. Host reconstruction supplies the
    # ProductControl-owned runtime identity; the reader rejects an unbound build.
    product_session_id = str(config.get("product_session_id", "") or os.environ.get("LINGTU_PRODUCT_SESSION_ID", ""))
    domain_id = int(os.environ.get("LINGTU_DDS_DOMAIN_ID", "0") or "0")
    builder.add(
        endpoint, alias="measurements",
        product_session_id=product_session_id,
        session_id=config["measurement_session_id"],
        domain_id=domain_id,
        required_sensor_ids=config["measurement_sensor_ids"],
    )
    generations = {(item["model_generation"], item["reset_generation"]) for item in config["measurement_sensors"].values()}
    if len(generations) != 1:
        raise ValueError("sensor controls require one compiled model/reset generation")
    model_generation, reset_generation = next(iter(generations))
    controller = stack_module("sensor_controls", "native_dds", seed_group="camera_sim",
                              fallback="drivers.sim.measurement.control_module.SensorControlModule")
    return builder.add(controller, alias="sensor.controls", product_session_id=product_session_id,
                       session_id=config["measurement_session_id"], domain_id=domain_id,
                       sensor_ids=config["measurement_sensor_ids"], model_generation=model_generation,
                       reset_generation=reset_generation)
