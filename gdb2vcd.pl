#!/usr/bin/perl

use strict;

use feature "switch";
use POSIX();

sub main{
	my @handles;
	my @vars;
	my %dumpvars;

	open(SRC, "<", $_[0]) or die $!;
	while(<SRC>){
		my @args = split(",");
		my $type = $args[0];
		given($type)
		{
			when('%create'){
				my $handle = $args[1];
				my $name = substr($args[2]=~s/ /_/gr, 1, -2);
				push @handles, $handle;
				push @vars, "\$var wire 1 $handle $name \$end";
			}
			when('%in'){
				scalar @args == 3 or continue;
				
				my $time = POSIX::floor($args[1]);
				my $dump = "b1 $args[2]";
				$dumpvars{$time} or $dumpvars{$time} = [];
				push @{$dumpvars{$time}}, $dump;
			}
			when('%out'){
				scalar @args == 3 or continue;
				
				my $time = POSIX::floor($args[1]);
				my $dump = "b0 $args[2]";
				$dumpvars{$time} or $dumpvars{$time} = [];
				push @{$dumpvars{$time}}, $dump;
			}
		}
	}
	close(SRC);
	
	open(DST, ">", "sched.vcd");
	
	# Header
	print DST "\$version\n\$end\n";
	print DST "\$timescale 1 ms\n\$end\n";
	
	# Vars
	for(@vars){
		print DST "$_\n";
	}
	
	# Dumpvars
	
		print DST "#0\n";
	for(@handles){
		print DST "b0 $_\n";
	}
	for my $time (sort {$a <=> $b} keys %dumpvars){
		print DST "#$time\n";
		for(@{$dumpvars{$time}}){
			print DST "$_\n";
		}
	}
	
	close(DST);
}

main($ARGV[0]);
