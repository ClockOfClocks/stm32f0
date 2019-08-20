
# ClockOfClocks (stm32f0) 


## How to use?

1. Install platform.io

2. Download [framework-cmsis.tar.gz](https://github.com/platformio/platform-ststm32/files/617820/framework-cmsis.tar.gz) from [issue#26](https://github.com/platformio/platform-ststm32/issues/26), unpack and copy `variants/stm32f0` directory to `packages/framework-cmsis/variants/ststm32/` under platform.io [core dir](https://docs.platformio.org/en/latest/projectconf/section_platformio.html#projectconf-pio-core-dir)

3. Create `boards` directory under platform.io [core dir](https://docs.platformio.org/en/latest/projectconf/section_platformio.html#projectconf-pio-core-dir) and copy `support/stm32f030c8t6.json` file to it;
4. Copy `support/stm32f030c8t6.cfg` to `.platformio/packages/tool-openocd/scripts/board`.


## Documents

Stm32f030 Reference Manual:  
https://www.st.com/content/ccc/resource/technical/document/reference_manual/cf/10/a8/c4/29/fb/4c/42/DM00091010.pdf/files/DM00091010.pdf/jcr:content/translations/en.DM00091010.pdf