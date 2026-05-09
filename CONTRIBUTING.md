# Contributing

Patches are welcome.

Please keep changes small and easy to test.

## Before Opening a Pull Request

Run the host tests:

```sh
tests/run_host_tests.sh
```

If you have the Pico SDK installed, also run:

```sh
cmake --build build
```

## Issues

Use issues for repeatable bugs, hardware notes, and confirmed compatibility problems.

Please include:

* Hardware used
* Wiring changes
* Firmware commit
* What you expected
* What happened
* Steps to repeat it

General wiring and setup questions are better kept in discussions if that is enabled.

## Code Style

* C++17
* Pico SDK APIs
* Small modules
* Nonblocking main loop code
* No motor or servo output code
* Host tests for packet formats and pure logic

Keep MSP payload changes documented in `src/msp.cpp`.

## Hardware Changes

Any hardware behavior change should include bench test notes.

For DJI O4 changes, note the goggles model, air unit firmware, and whether Canvas Mode worked.
