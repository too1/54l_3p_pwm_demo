cd app
west build -b nrf54l15pdk/nrf54l15/cpuapp -S nordic-flpr
cd ..
cd vpr
west build -b nrf54l15pdk/nrf54l15/cpuflpr
cd ..
