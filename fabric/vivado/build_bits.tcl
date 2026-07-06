set root $::env(HOME)/kria-reverb
open_project $root/reverb_bd/reverb_bd.xpr
set bd [get_files design_1.bd]
generate_target all $bd
make_wrapper -files $bd -top -import
set_property top design_1_wrapper [current_fileset]
update_compile_order -fileset sources_1
read_xdc $root/fan.xdc
set_property used_in_synthesis false [get_files $root/fan.xdc]
set_property STEPS.PLACE_DESIGN.ARGS.DIRECTIVE ExtraTimingOpt [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]
set_property STEPS.WRITE_BITSTREAM.ARGS.BIN_FILE true [get_runs impl_1]
launch_runs impl_1 -to_step write_bitstream -jobs 16
wait_on_run impl_1
puts "BITS_WNS [get_property STATS.WNS [get_runs impl_1]]"
write_hw_platform -fixed -include_bit -force $root/reverb.xsa
set binf [glob -nocomplain $root/reverb_bd/reverb_bd.runs/impl_1/*wrapper.bin]
if {$binf ne ""} { file copy -force [lindex $binf 0] $root/reverb.bit.bin }
puts "BITSTREAM_DONE"
