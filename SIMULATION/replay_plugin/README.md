# Replay plugin for MADS

This is a Source plugin for [MADS](https://github.com/MADS-NET/MADS). 

This plugin allows to *replay* in MADS a CSV log file. It reads a CSV file and produces a JSON message for each line.

The CSV header line is interpreted as a list of *keypaths*. For example, if a column is `/IMU/0/accel/x` (notice the **mandatory leading slash**), the corresponding value gets into the JSON object `{"IMU": [{"accel": {"x": 123.456}}]}`. 

An alternative keypaths syntax uses slashes as separators. For example, `/IMU/0/accel/x` can also be written as `IMU[0].accel.x`. Note that this sytax is less efficient and mostly supported for compatibility reasons.

A MADS message is emitted for each line, at a frequency set by the agent period (from command line `-p` or from INI file)

*Required MADS version: 1.3.5.*


## Supported platforms

Currently, the supported platforms are:

* **Linux** 
* **MacOS**
* **Windows**


## Installation

Linux and MacOS:

```bash
cmake -Bbuild -DCMAKE_INSTALL_PREFIX="$(mads -p)"
cmake --build build -j4
sudo cmake --install build
```

Windows:

```powershell
cmake -Bbuild -DCMAKE_INSTALL_PREFIX="$(mads -p)"
cmake --build build --config Release
cmake --install build --config Release
```


## INI settings

The plugin supports the following settings in the INI file:

```ini
[replay]
csv_file = "path/to/file.csv"
loop = true # once the end of file has been reached, start again from beginning
```

All settings are optional; if omitted, the default values are used.




---