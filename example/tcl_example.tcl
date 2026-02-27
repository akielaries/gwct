# example.tcl GWCT Tcl script example
# much of this is very specific to my setup, interchange as you need. you get
# the idea...
#
# Run from the gwct shell:
#   gwct> tcl example.tcl
#
# Run non-interactively from the command line:
#   gwct --device mega60 --remote 192.168.1.10 --tcl example.tcl

set base 0x60000000

puts [memrd $base]

# load a bitstream...
#program ../mega_60k_test/impl/pnr/mega_60k_test.fs

puts ""
puts "script done"
