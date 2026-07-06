open_project -reset reverb_ip
set_top reverb_kernel
add_files /home/jak/kria-reverb/reverb_kernel.cpp -cflags "-I/home/jak/kria-reverb"
open_solution -reset sol1
set_part {xck26-sfvc784-2LV-c}
create_clock -period 5 -name default
csynth_design
export_design -format ip_catalog -rtl verilog -output /home/jak/kria-reverb/reverb_kernel_ip.zip
exit
