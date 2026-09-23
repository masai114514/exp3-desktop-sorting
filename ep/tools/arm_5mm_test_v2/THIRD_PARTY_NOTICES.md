# Third-party components

The `vendor/robomaster` directory is copied from the user's supplied archive,
which identifies it as the Python portion of DJI RoboMaster SDK 0.1.1.68.
Original copyright headers are retained; the SDK uses Apache License 2.0.
This is a local task-specific modification, not an official DJI release.

Source: https://github.com/dji-sdk/RoboMaster-SDK

Original upload modifications: camera import removed in robot.py.
Additional local modifications: remove camera construction and camera registration
in the two `_scan_modules` methods, because the import-only trimming left unresolved
camera references during EP initialization. Video/camera/Tello are not supported by this checker.

`vendor/netaddr-1.3.0-py3-none-any.whl` is copied without modification from the upload;
its own license and metadata are inside the wheel.

`vendor/netifaces.py` is a small import shim, not the upstream netifaces package.
The supported EP AP path does not enumerate NICs; unsupported enumeration raises
an explicit exception. Do not install this shim into system site-packages.

No external dependencies are installed or globally replaced by this tool.
