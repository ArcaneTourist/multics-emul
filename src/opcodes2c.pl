#!/usr/bin/perl -w

#   Copyright (c) 2007-2013 Michael Mondy
#
#   This software is made available under the terms of the
#   ICU License -- ICU 1.8.1 and later.     
#   See the LICENSE file at the top-level directory of this distribution and
#   at http://example.org/project/LICENSE.

use strict;

my @ops0 = load_ops(0);
my @ops1 = load_ops(1);

print "extern char *op0text[512];\n";
print "extern char *op1text[512];\n";
print "extern char *opcodes2text[1024];\n";
print "\n";

print "// Opcodes with low bit (bit 27) == 0.  Enum value is value of upper 9 bits.\n";
printf "typedef enum {\n";
dump_enums(0, '0', @ops0);
print "} opcode0_t;\n";
print "\n";

print "// Opcodes with low bit (bit 27) == 1.  Enum value is value of upper 9 bits.\n";
printf "typedef enum {\n";
dump_enums(0, '1', @ops1);
print "} opcode1_t;\n";
print "\n";

printf "#ifndef NULL\n";
printf "#define NULL ((void*) 0)\n";
printf "#endif\n";

print "char *op0text[512] = {\n";
print "\t// index by upper 9 bits of those opcodes with bit 27 == 0\n";
dump_txt(@ops0);
print "};\n";
print "\n";

printf "char *op1text[512] = {\n";
print "\t// index by upper 9 bits of those opcodes with bit 27 == 1\n";
dump_txt(@ops1);
print "};\n";
print "\n";

printf "char *opcodes2text[1024] = {\n";
print "\t// index by all 10 bits of all opcodes\n";
dump_all_txt(\@ops0, \@ops1);
#dump_txt(@ops0);
#print ",\n";
#dump_txt(@ops1);
print "};\n";
print "\n";

exit;

# =============================================================================

sub dump_enums {
	my $show_unused = shift;
	my $tag = shift;
	my @ops = @_;
	my $i = 0;
	my $last;
	# hack for C++
	foreach my $op (@ops) {
		$last = $op unless ($op eq '-');
	}
	foreach my $op (@ops) {
		# my $val = $i << 1;
		if ($op eq '-') {
			if ($show_unused) {
				printf "\t\t     // 0%03o unused (%d decimal)\n", $i, $i;
			}
		} else {
			my $comma = ($op eq $last) ? "" : ",";
			printf "\topcode%s_%s%*s = 0%03o%s // (%d decimal)\n",
				$tag, $op, 6-length($op), "", $i, $comma, $i;
		}
		++ $i;
	}
}

# =============================================================================

sub dump_txt {
	my @ops = @_;
	my $i = 0;
	foreach my $op (@ops) {
		# my $val = $i << 1;
		printf "%s", ($i % 8 == 0) ? "\t" : " ";
		my $term = ($i == $#ops) ? "" : ",";
		if ($op eq '-') {
			printf "%-8s", "NULL" . $term;
		} else {
			printf "%-8s", "\"$op\"" . $term;
		}
		++ $i;
		print "\n" if ($i % 8 == 0);
	}
}

# =============================================================================

sub dump_all_txt {
	my $ops0ref = shift;
	my $ops1ref = shift;

	my @ops1 = @$ops1ref;
	my $i = 0;
	foreach my $op0 (@$ops0ref) {
		# printf "%s", ($i % 8 == 0) ? "\t" : " ";
		# First print op0 which has a 10th bit of zero
		if ($i % 8 == 0) {
			print "\t";
		} else {
			print " ";
		}
		if ($op0 eq '-') {
			printf "%-8s", "NULL,";
		} else {
			printf "%-8s", "\"$op0\",";
		}
		print " ";
		# Second, print op1 which has a 10th bit of one
		my $op1 = shift @ops1;
		my $term = ($i/2 == $#$ops0ref) ? "" : ",";
		if ($op1 eq '-') {
			printf "%-8s", "NULL" . $term;
		} else {
			printf "%-8s", "\"$op1\"" . $term;
		}
		$i += 2;
		print "\n" if ($i % 8 == 0);
	}
}
# =============================================================================

sub load_ops {
	my $bit = shift;
	my $alpha = ($bit eq '0' || $bit == 0) ? '0' : '1';

	my $f = "opcodes$bit.txt";
	my @ops;
	open(OPS, "< $f") or
		die "$0: Cannot open $f: $!\n";
	while(<OPS>) {
		chomp;
		next if (/^\s*#/);
		next if (/^\s*$/);
		my @a = split;
		if ($#a != 15) {
			die "$0: Expecting 16 opcodes on line $.\n";
		}
		push @ops, @a;
	}
	close(OPS) or
		die "$0: Error reading $f: $!\n";
	if ($#ops != 511) {
		die "$0: Expecting 512 opcodes in $f.\n";
	}
	return @ops;
}
