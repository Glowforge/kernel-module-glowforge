#!/usr/bin/perl

# (c) Eli Billauer, 2011. http://billauer.co.il

# The code contained herein is licensed under the GNU General Public
# License. You may obtain a copy of the GNU General Public License
# Version 2 or later at the following locations:
#
# http://www.opensource.org/licenses/gpl-license.html
# http://www.gnu.org/copyleft/gpl.html

use warnings;
use strict;
use mx51_sdma_set;

my $hex_per_row = 8;
my @source = <>;
my @objectcode;
my $context = {}; # Ref to empty hash

my $instructref = \%mx51_sdma_set::instructions;
my %labels;

my $generate_code = 1;

print "/*\n";

assemble(1); # Assembly pass 1

@objectcode = (); # Clean before second pass

$context->{'labels'} = \%labels;
$context->{'objectcode'} = \@objectcode;

assemble(2); # Assembly pass 2

die("\n*** No code generated due to error(s) ***\n")
  unless ($generate_code);

if (0) { # Enable this for 16-bit hex code

  my @hexcode = map { sprintf '0x%04x', $_ } @objectcode;
  
  print "\n------------ CUT HERE -----------\n\n";
  print "(Remember to write the data as BIG endian)\n\n";
  print "static const int sdma_code_length = ".scalar(@objectcode).";\n";
  print "static const u16 sdma_code[".scalar(@objectcode)."] = {\n";
  
  while (@hexcode) {
    print "  ".(join ', ', splice @hexcode, 0, $hex_per_row).",\n";
  }
  
  print "};\n";
}

if (1) { # This enabled for 32-bit hex code (easier!)
  my @rawcode = @objectcode;
  my @hexcode;

  while (@rawcode) {
    my $word = sprintf '0x%04x', shift @rawcode;
    $word .= sprintf '%04x', (shift @rawcode || 0);
    push @hexcode, $word;
  }
  print "*/\n";
#  print "\n------------ CUT HERE -----------\n\n";
#  print "static const int sdma_code_length = ".scalar(@hexcode).";\n";
#  print "static const u32 sdma_code[".scalar(@hexcode)."] = {\n";
  
  while (@hexcode) {
    print "  ".(join ', ', splice @hexcode, 0, $hex_per_row).",\n";
  }
  
#  print "};\n"; 

}

sub assemble {
  my ($pass) = @_;

  my $listing = ($pass > 1);

  my $linenum = 1;

  foreach my $line (@source) {
    chomp $line;
    my $x = $line;
    my $binary;
    my $error;

    $x =~ s/[;\#].*//; # Remove commments

    if ($x =~ /^[ \t]*(\w+):(.*)/) {
      $x = $2;
      my $label = lc $1;

      if ($pass == 1) {
	die("ERROR in line $linenum: Label \"$label\" previously defined\n")
	  if (defined $labels{$label});
	
	$labels{$label} = scalar @objectcode;
      }
    }

    ($x) = ($x =~ /^[ \t]*(.*?)[ \t\r\n]*$/); # Remove surrounding whitespace

    goto bottom if ($x eq '');

    $x = lc $x; # Case insensitive

    my ($opcode, $args) = ($x =~ /([^ \t]+)[ \t]*(.*)/);

    eval {
      my $opcoderef = $instructref->{$opcode};

      die("Unknown opcode \"$opcode\"\n")
	unless (defined $opcoderef);

      $args =~ s/[ \t]*([,\(\)])[ \t]*/$1/ge; # Remove whitespaces around delimiters

      $binary = $opcoderef->[0];
      $binary |= &{$opcoderef->[1]}($args, $context)
	if ($pass > 1);
    };

    if ($@) {
      $error = "ERROR in line $linenum: $@";
      $generate_code = 0;
      goto bottom;
    }

    push @objectcode, $binary;

    if ($listing) {
      printf "%04x %04x (%s) | %s\n", ($#objectcode), $binary,
	zero_one($binary), $line;
    }

    $linenum++;
    next;
  bottom:
    if ($listing) {
      print "                             | $line\n";
      print "$error\n" if (defined $error);
    }

    $linenum++;
  }
}

sub zero_one {
  my $val = shift;
  my $out = '';

  for (my $i=0; $i<16; $i++) {
    $out .= ($val & 0x8000) ? '1' : '0';
    $val <<= 1;
  }

  return $out;
}
