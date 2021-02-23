# (c) Eli Billauer, 2011. http://billauer.co.il

# The code contained herein is licensed under the GNU General Public
# License. You may obtain a copy of the GNU General Public License
# Version 2 or later at the following locations:
#
# http://www.opensource.org/licenses/gpl-license.html
# http://www.gnu.org/copyleft/gpl.html

package mx51_sdma_set;

use warnings;
use strict;

our %instructions = ( done => [ 0x0000, \&j8x3 ],
		      yield => [ 0x0000, \&noargs ],
		      yieldge => [ 0x0100, \&noargs ],
		      notify => [ 0x0001, \&j8x3 ],
		      softbkpt => [ 0x0005, \&noargs ],
		      ret => [ 0x0006, \&noargs ],
		      clrf => [ 0x0007, \&f8x2 ],
		      illegal => [ 0x0707, \&noargs ],
		      jmpr => [ 0x0008, \&r8x3 ],
		      jsrr => [ 0x0009, \&r8x3 ],
		      ldrpc => [ 0x000a, \&r8x3 ],
		      revb => [ 0x0010, \&r8x3 ],
		      revblo => [ 0x0011, \&r8x3 ],
		      rorb => [ 0x0012, \&r8x3 ],
		      ror1 => [ 0x0014, \&r8x3 ],
		      lsr1 => [ 0x0015, \&r8x3 ],
		      asr1 => [ 0x0016, \&r8x3 ],
		      lsl1 => [ 0x0017, \&r8x3 ],
		      bclri => [ 0x0020, \&r8x3_n0x5 ],
		      bseti => [ 0x0040, \&r8x3_n0x5 ],
		      btsti => [ 0x0060, \&r8x3_n0x5 ],
		      mov => [ 0x0088, \&r8x3_s0x3 ],
		      xor => [ 0x0090, \&r8x3_s0x3 ],
		      add => [ 0x0098, \&r8x3_s0x3 ],
		      sub => [ 0x00a0, \&r8x3_s0x3 ],
		      or => [ 0x00a8, \&r8x3_s0x3 ],
		      andn => [ 0x00b0, \&r8x3_s0x3 ],
		      and => [ 0x00b8, \&r8x3_s0x3 ],
		      tst => [ 0x00c0, \&r8x3_s0x3 ],
		      cmpeq => [ 0x00c8, \&r8x3_s0x3 ],
		      cmplt => [ 0x00d0, \&r8x3_s0x3 ],
		      cmphs => [ 0x00d8, \&r8x3_s0x3 ],
		      cpshreg => [ 0x06e2, \&noargs ],
		      ldi => [ 0x0800, \&r8x3_i0x8 ],
		      xori => [ 0x1000, \&r8x3_i0x8 ],
		      addi => [ 0x1800, \&r8x3_i0x8 ],
		      subi => [ 0x2000, \&r8x3_i0x8 ],
		      ori => [ 0x2800, \&r8x3_i0x8 ],
		      andni => [ 0x3000, \&r8x3_i0x8 ],
		      andi => [ 0x3800, \&r8x3_i0x8 ],
		      tsti => [ 0x4000, \&r8x3_i0x8 ],
		      cmpeqi => [ 0x4800, \&r8x3_i0x8 ],
		      ld => [ 0x5000, \&r8x3_d3x5_b0x3 ],
		      st => [ 0x5800, \&r8x3_d3x5_b0x3 ],
		      ldf => [ 0x6000, \&r8x3_u0x8 ],
		      stf => [ 0x6800, \&r8x3_u0x8 ],
		      loop => [ 0x7800, \&f8x2_n0x8 ],
		      bf => [ 0x7c00, \&p0x8 ],
		      bt => [ 0x7d00, \&p0x8 ],
		      bsf => [ 0x7e00, \&p0x8 ],
		      bdf => [ 0x7f00, \&p0x8 ],
		      jmp => [ 0x8000, \&a0x14 ],
		      jsr => [ 0xc000, \&a0x14 ],
		    );

# General-purpose functions
sub numsplit {
  my ($arg, $num) = @_;

  my @args = split /,/, $arg;
  my $real_num = scalar @args;


  die("Expected $num arguments, found $real_num\n")
    unless ($real_num == $num);

  return @args;
}

sub one_register {
  my ($reg) = @_;
  my ($num) = ($reg =~ /^r(\d)$/);

  die("Expected register argument of the form r0, r1, ... r7 (found \"$reg\")\n")
    unless ((defined $num) && ($num < 8));

  return $num;
}

sub get_number {
  my ($arg) = @_;

  return undef unless (defined $arg); # Can this ever happen?

  return hex($1)
    if ($arg =~ /^0x([0-9a-fA-F]+)$/);

  return undef
    unless ($arg =~ /^-{0,1}\d+$/);

  return $arg;
}

sub numeric_arg {
  my ($arg, $bits) = @_;

  my $num = get_number($arg);

  die("Argument \"$arg\" is not a legal number\n")
    unless (defined $num);

  die("Argument is out of range (too big)\n")
    if ($num >= (1 << $bits));

  die("Negative argument is out of range (to small)\n")
    if ($num < -(1 << ($bits-1)));

  return $num;
}

# If it can be a label, it's up to the caller to check range, since a
# label can be out of range anyhow

sub numeric_arg_or_label {
  my ($arg, $bits, $context) = @_;

  my $num = get_number($arg);

  return $num if (defined $num);

  my $labeladdr = $context->{'labels'}->{$arg};

  die("Argument \"$arg\" is not a legal number nor a known label\n")
    unless (defined $labeladdr);

  my $nextaddr = scalar @{$context->{'objectcode'}} + 1;

  return ($labeladdr - $nextaddr);
}


# Argument mangling functions

sub a0x14 {
  my ($args, $context) = @_;
  my $a = numeric_arg(numsplit($args,1), 14);

  return ( ($a & 16383) << 0);
}

sub f8x2 {
  my ($args, $context) = @_;
  my $f = numeric_arg(numsplit($args,1), 2);

  return ( ($f & 3) << 8);
}

sub f8x2_n0x8 {
  my ($args, $context) = @_;
  my ($f, $n);

  my @vals = numsplit($args,2);

  $n = numeric_arg_or_label($vals[0], 8, $context);
  $f = numeric_arg($vals[1], 2);

  die("Loop label must be after the statement\n")
    if ($n < 0);

  die("An empty loop is not legal\n")
    if ($n == 0);

  die("Loop label is too far away\n")
    if ($n > 255);

  return ( ($f & 3) << 8) | ( ($n & 255) << 0);
}

sub j8x3 {
  my ($args, $context) = @_;
  my $j = numeric_arg(numsplit($args,1), 3);

  return ( ($j & 7) << 8);
}

sub noargs {
  my ($args, $context) = @_;

  die("This opcode takes no arguments\n")
    if (length $args);

  return 0;
}

sub p0x8 {
  my ($args, $context) = @_;
  my $p = numeric_arg_or_label(numsplit($args,1), 8, $context);

  die("Branch address is out of range\n")
    if (($p > 127) || ($p < -128));

  return ( ($p & 255) << 0);
}

sub r8x3 {
  my ($args, $context) = @_;
  my $r = one_register(numsplit($args,1));

  return ( ($r & 7) << 8);
}

sub r8x3_d3x5_b0x3 {
  my ($args, $context) = @_;

  my @vals = ($args =~ /^(\w+),\((\w+),(\w+)\)$/);

  die("Expected arguments in the format ra, (rb, displacement)\n")
    unless (@vals);

  my $r = one_register($vals[0]);
  my $d = numeric_arg($vals[2], 5);
  my $b = one_register($vals[1]);

  die("Negative displacement is illegal\n")
    if ($d < 0);

  return ( ($r & 7) << 8) | ( ($d & 31) << 3) | ( ($b & 7) << 0);
}

sub r8x3_i0x8 {
  my ($args, $context) = @_;

  my @vals = numsplit($args,2);

  my $r = one_register($vals[0]);
  my $i = numeric_arg($vals[1], 8);

  return ( ($r & 7) << 8) | ( ($i & 255) << 0);
}

sub r8x3_n0x5 {
  my ($args, $context) = @_;

  my @vals = numsplit($args,2);

  my $r = one_register($vals[0]);
  my $n = numeric_arg($vals[1], 5);

  return ( ($r & 7) << 8) | ( ($n & 31) << 0);
}

sub r8x3_s0x3 {
  my ($args, $context) = @_;

  my @vals = numsplit($args,2);

  my $r = one_register($vals[0]);
  my $s = one_register($vals[1]);
  
  return ( ($r & 7) << 8) | ( ($s & 7) << 0);
}

sub r8x3_u0x8 {
  my ($args, $context) = @_;

  my @vals = numsplit($args,2);

  my $r = one_register($vals[0]);
  my $u = numeric_arg($vals[1], 8);

  return ( ($r & 7) << 8) | ( ($u & 255) << 0);
}

1;
