# example.tcl GWCT Tcl script example
#
# Run from the gwct shell:
#   gwct> tcl example.tcl
#
# Run non-interactively from the command line:
#   gwct --device mega60 --remote 192.168.1.10 --tcl example.tcl
#
# Available commands:
#   mrd  <addr>          - read 32-bit register, returns "0xXXXXXXXX"
#   mwr  <addr> <data>   - write 32-bit register
#   dump <addr> <count>  - hex dump, returns formatted string
#   load_bitstream <f>   - program .fs file, returns 0 (ok) or 1 (fail)
#   sleep_ms <ms>        - sleep
#   memrd / memwr        - aliases for mrd / mwr
#   All standard Tcl (puts, set, if, proc, for, while, expr, etc.)
#
# Addresses accept hex (0x...) and arithmetic expressions via expr.

# ---------------------------------------------------------------------------
# 1. Basic register read
# ---------------------------------------------------------------------------
set base 0x60000000

set magic [mrd $base]
puts "magic    = $magic"

if { $magic eq "0xDEADBEEF" } {
    puts "sysinfo magic OK"
} else {
    puts "sysinfo magic FAIL - expected 0xDEADBEEF, got $magic"
}

puts ""
puts "sysinfo registers:"
for { set i 0 } { $i < 4 } { incr i } {
    set addr [expr { $base + $i * 4 }]
    set val  [mrd $addr]
    puts [format "  0x%08X : %s" $addr $val]
}

puts ""
puts "hex dump:"
puts [dump $base 8]


# load a bitstream...
#load ../mega_60k_test/impl/pnr/mega_60k_test.fs

puts ""
puts "script done"
