# Getting Started with WIZ5XXSR-RP-C

WIZ5XXSR-RP-C is used for WIZ500SR-RP, WIZ505SR-RP and WIZ510SR-RP.

These sections will provide informations about WIZ500SR-RP, WIZ505SR-RP and WIZ510SR-RP, which are S2E(Serial to Ethernet) modules using WIZ5XXSR-RP-C, and how to configure development environment to develop and modify WIZ5XXSR-RP-C.

- [**Development environment configuration**](#development_environment_configuration)
- [**Hardware requirements**](#hardware_requirements)



<a name="development_environment_configuration"></a>
## Development environment configuration

To develop and modify WIZ5XXSR-RP-C, the development environment must be configured so that RP2040 can be used.

WIZ5XXSR-RP-C was developed by configuring the development environment for **Windows**, When configuring the development environment, refer to the '**9.2. Building on MS Windows**' section of '**Getting started with Raspberry Pi Pico**' document below to configure the development environment.

- [**Getting started with Raspberry Pi Pico**][link-getting_started_with_raspberry_pi_pico]

If you want development environments other than the development environment for Windows, note that you can find other ways to configure development environment in **'Chapter 9. Building on other platforms'**  section of the document above.



<a name="hardware_requirements"></a>
## Hardware requirements

The WIZ5XXSR-RP-C use **WIZ500SR-RP**, **WIZ505SR-RP** and **WIZ510SR-RP** - S2E(Serial to Ethernet) modules built on [**RP2040**][link-rp2040] and WIZnet's [**W5100S**][link-w5100s] ethernet chip.

For detailed information about each product using WIZ5XXSR-RP-C, refer to the link below.

- [**WIZ500SR-RP**][link-wiz500sr-rp]

<p align="center"><img src="https://github.com/Wiznet/W5XXSR-RP-C/blob/main/static/images/getting_started/wiz500sr-rp_main.png"></p>

- [**WIZ505SR-RP**][link-wiz505sr-rp]

<p align="center"><img src="https://github.com/Wiznet/W5XXSR-RP-C/blob/main/static/images/getting_started/wiz505sr-rp_main.png"></p>

- [**WIZ510SR-RP**][link-wiz510sr-rp]

<p align="center"><img src="https://github.com/Wiznet/W5XXSR-RP-C/blob/main/static/images/getting_started/wiz510sr-rp_main.png"></p>



<!--
Link
-->

[link-getting_started_with_raspberry_pi_pico]: https://datasheets.raspberrypi.org/pico/getting-started-with-pico.pdf
[link-rp2040]: https://www.raspberrypi.org/products/rp2040/
[link-w5100s]: https://docs.wiznet.io/Product/iEthernet/W5100S/overview
[link-wiz500sr-rp]: https://docs.wiznet.io/Product/S2E-Module/WIZ5xxSR-RP-Series/WIZ500SR-RP/overview
[link-wiz505sr-rp]: https://docs.wiznet.io/Product/S2E-Module/WIZ5xxSR-RP-Series/WIZ505SR-RP/overview
[link-wiz510sr-rp]: https://docs.wiznet.io/Product/S2E-Module/WIZ5xxSR-RP-Series/WIZ510SR-RP/overview
[link-wiz500sr-rp_main]: https://github.com/Wiznet/W5XXSR-RP-C/blob/main/static/images/getting_started/wiz500sr-rp_main.png
[link-wiz505sr-rp_main]: https://github.com/Wiznet/W5XXSR-RP-C/blob/main/static/images/getting_started/wiz505sr-rp_main.png
[link-wiz510sr-rp_main]: https://github.com/Wiznet/W5XXSR-RP-C/blob/main/static/images/getting_started/wiz510sr-rp_main.png
