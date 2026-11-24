"""Start the field owner from a checked deployment bundle."""

from run_service_fleet import field_settings

from roport_control_server.executor.fleet_runtime import main

if __name__ == "__main__":
    main(field_loader=field_settings)
