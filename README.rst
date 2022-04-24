Introduction
============


.. image:: https://readthedocs.org/projects/adafruit-circuitpython-ov7670/badge/?version=latest
    :target: https://docs.circuitpython.org/projects/ov7670/en/latest/
    :alt: Documentation Status


.. image:: https://raw.githubusercontent.com/adafruit/Adafruit_CircuitPython_Bundle/main/badges/adafruit_discord.svg
    :target: https://adafru.it/discord
    :alt: Discord


.. image:: https://github.com/adafruit/Adafruit_CircuitPython_OV7670/workflows/Build%20CI/badge.svg
    :target: https://github.com/adafruit/Adafruit_CircuitPython_OV7670/actions
    :alt: Build Status


.. image:: https://img.shields.io/badge/code%20style-black-000000.svg
    :target: https://github.com/psf/black
    :alt: Code Style: Black

CircuitPython driver for OV7670 cameras

.. warning::
    This module requires the CircuitPython ``imagecapture`` module which is only in the unreleased development version ("Absolute Newest") of CircuitPython and is only supported on specific boards.

Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://circuitpython.org/libraries>`_
or individual libraries can be installed using
`circup <https://github.com/adafruit/circup>`_.

.. :: Describe the Adafruit product this library works with. For PCBs, you can also add the image from the assets folder in the PCB's github repo.
.. :: `Purchase one from the Adafruit shop <http://www.adafruit.com/products/>`_



Usage Example
=============

On an Adafruit Metro M4 Grand Central, capture a 40x30 image into a buffer:

.. code-block:: python3

    import board
    from adafruit_ov7670 import OV7670

    cam = OV7670(
        bus,
        data_pins=[board.PCC_D0, board.PCC_D1, board.PCC_D2, board.PCC_D3, board.PCC_D4, board.PCC_D5, board.PCC_D6, board.PCC_D7],
        clock=board.PCC_CLK,
        vsync=board.PCC_DEN1,
        href=board.PCC_DEN2,
        mclk=board.D29,
        shutdown=board.D39,
        reset=board.D38,
    )
    cam.size = OV7670_SIZE_DIV16

    buf = bytearray(2 * cam.width * cam.height)

    cam.capture(buf)

Documentation
=============

API documentation for this library can be found on `Read the Docs <https://docs.circuitpython.org/projects/ov7670/en/latest/>`_.

For information on building library documentation, please check out `this guide <https://learn.adafruit.com/creating-and-sharing-a-circuitpython-library/sharing-our-docs-on-readthedocs#sphinx-5-1>`_.

Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/adafruit/Adafruit_CircuitPython_OV7670/blob/main/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.
