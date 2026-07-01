# mo

`mo` is the Raspberry Pi 5 / Fedora Linux host program for the RV-10 display and support tools.
The primary application drives a BT815 / EVE based display and communicates with EFI, vibration-sensing,
lidar, sound, and utility interfaces.

## Layout

```text
.
 src/mo/             Main `mo` application sources
 src/eve/            BT815 / EVE display interface and FTDI bridge code
 src/vibsense/       C vibration analysis support library
 include/            Public/project headers
 include/eve/        EVE display headers
 include/vibsense/   vibsense library headers
 tools/              Utility programs (`dl`, `load`, image generators, EVE test main)
 scripts/            Shell helper scripts
 assets/             Images, converted 565 images, sounds, fonts, screenshots
 config/             Example runtime configuration files
 runtime/            Default local logs and screenshots directory
 udev/               udev rules for target systems
 docs/               Notes and vendor/reference documents
 vib_analysis/       Python vibration-analysis tools
```

## Build

Install the usual development dependencies first.  On Raspberry Pi OS 64-bit these include:

```sh
sudo apt install build-essential pkg-config libconfig-dev espeak libftdi1-dev gpiod libgpiod-dev libfreetype6-dev libjpeg-dev xxd
```

For Fedora development hosts:

```sh
sudo dnf install gcc make libconfig-devel espeak-ng-devel libftdi-devel   freetype-devel libjpeg-turbo-devel xxd
```

Then build from the repository root:

```sh
make
```

The top-level Makefile builds:

* `./mo` - the main application
* `./dl` - log/diagnostic utility
* `./make_jpg` - splash/image helper
* `build/lib/lib_eve.a`
* `build/lib/libvibsense.a`

Useful targets:

```sh
make all
make tools
make test
make clean
make cleanrun
make cleanlogs
make logs
```

## Runtime notes

The main program is intended to run from the repository root.  The application looks at the following files:

* `config/mo.config`
* `assets/images/default_splash.jpg`
* `assets/images/startup_base.jpg`
* `assets/sounds/*.wav`
* `runtime/logs`                                    (Log data written here)
* `runtime/screenshots`                             (Screenshots written here)

Raspberry Pi setup notes are included in `docs/README.pi`.

Vibration sensor code is still under development.

## Licensing

Project source files have the MIT License header and the full text is in `LICENSE`.

Not everything in this repository is necessarily MIT-licensed. In particular, bundled fonts retain their own
upstream licensing or distribution terms. See assets/fonts/cinzel/OFL.txt.
