# SCHUNK FTS Library
A Low-level communication library for SCHUNK's force-torque sensors.

## ROS2 installation
In a sourced environment

```bash
colcon build --packages-select schunk_fts_library
```

## Stand-alone installation

It's easiest to use _virtual environments_ to keep your python projects nicely separated.
Navigate inside this package and install it with

```bash
pip install .
```

You can then use pip's conventional tools, such as
```bash
pip show schunk_fts_library
```

Uninstall it with

```bash
pip uninstall schunk_fts_library
```
