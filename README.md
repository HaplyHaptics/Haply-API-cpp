# Haply C++ API

Haply C++ API is a cross-platform C++ library to interface with the opensource Haply Development Kit.

## Versions

  * Haply C++ API v0.1.0 (current): requires Haply Modular Haptic Development Kit Firmware v0.1+ for Haply Board v0.1+ (not yet compatible with Haply M0 and M3 boards), only working with 2-DOF pantograph mechanism.

## Installation

### Dependencies

Required:
* [CMake](http://www.cmake.org) (cross-platform build environment)
* [git](https://git-scm.com) (code versioning)

Optional:
* [doxygen](https://github.com/doxygen/doxygen) (documentation generator)
* [graphviz](http://graphviz.org) (diagram generator)

### Procedure

* Clone the git repository through SSH: `git clone git@github.com:HaplyHaptics/Haply-API-cpp.git`
* Update git submodules in the cloned repository: `git submodule update --init --recursive`
* Compile with CMake using your favorite integrated development environment or terminal. Turn CMake option `BUILD_DOC` to `ON` to enable documentation compilation (target: `doc`).

## Contributing

We are open to academic and industrial collaborations! 

### Code conventions

  * Format C++ code with `astyle --style=linux` (see [astyle doc](http://astyle.sourceforge.net/))

### Contact

Please contact us through your prefered way among the following!
  * Open an issue on github: https://github.com/HaplyHaptics/Haply-API-cpp/issues
  * Create a pull request on github: https://github.com/HaplyHaptics/Haply-API-cpp/pulls
  * Email us at [info@haply.co](mailto:info@haply.co).

## License

The Haply C++ API is a rewrite in C++ by Christian Frisson of the [Haply hAPI Java library](https://github.com/HaplyHaptics/Haply-hAPI/commit/30031d8f6d5a98815545e4f279a4b673161f91f0) written by Colin Gallacher and Steven Ding, therefore it inherits from the same license, updated [GPLv3 License](LICENSE). For serial communication, Haply C++ API uses [our fork](https://github.com/HaplyHaptics/serial) of [William Woodall's cross-platform serial port library](https://github.com/wjwwood/serial), with stripped-down dependencies (catkin and python removed), versioned as git submodule, also released under the terms of the MIT License.

If you want to reuse Haply C++ API for your research and development, you are free to do so as long as you comply with the terms of the [GPLv3 License](LICENSE).

## Authors

  * [Colin Gallacher](http://haply.co): original Haply Java hAPI
  * [Steven Ding](http://haply.co): original Haply Java hAPI
  * [Christian Frisson](http://frisson.re): port to Haply C++ API and CMake integration

## Reference

If you publish work based on this work (including ports and transpilations to other development languages), we kindly ask you to properly acknowledge our work by citing the publication related to the first showcase of the first demo built upon Haply C++ API: 

```
@inproceedings{FrissonFreesoundTrackerHAID2019,
 author = {Frisson, Christian and Gallacher, Colin and Wanderley, Marcelo M.},
 title = {Haptic techniques for browsing sound maps organized by similarity},
 booktitle = {International Workshop on Haptic and Audio Interaction Design},
 series = {HAID},
 year = {2019},
 month = {Mar},
 pages = {1},
 address = {Lille, France},
 url = {https://hal.archives-ouvertes.fr/hal-02050235},
 pdf = {https://hal.archives-ouvertes.fr/hal-02050235/file/demo3.pdf},
 hal_id = {hal-02050235},
 hal_version = {v1},
}
```

## Acknowledgements

This work has been undertaken as part of [Christian Frisson](http://frisson.re)'s postdoctoral fellowship with [Marcelo M. Wanderley](http://idmil.org) at McGill University, supported by [Mitacs](https://www.mitacs.ca) Elevate grant IT12555 co-funded by [Haply Robotics](http://haply.co).
