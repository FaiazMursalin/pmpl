#!/usr/bin/perl
# scipt to create xml files for all the boundaries of the protein
#########################################################################
$basexmlname = "base.xml";

@seeds=(
  '12345678',
);

@strategies=(
  'RapidFreePRM',
  'RapidQuickPRM',
  'PQPFreePRM',
  'PQPQuickPRM',
);

$seedsfilepath = 'seedsfile.txt';
$runxmlfilepath = 'runXmls.pl';
if(-e $seedsfilepath) {
  `rm $seedsfilepath`;
}
if(-e $runxmlfilepath) {
  `rm runXmls.pl`;
}
open SEEDS, '>>', "seedsfile.txt" or die "could not open seeds file $!\n";
open RUNXMLS, '>>', "runXmls.pl" or die "could not open xmls file $!\n";
foreach $strategy(@strategies) {
    foreach $seed(@seeds) {
       $copyname = $source.$strategy.".".$seed.".xml";
          if(-e $copyname) {
            `rm $copyname`;
          }
          `cp $basexmlname $copyname`;
          `replace _STRATEGY_ $strategy -- $copyname`;
          `replace _BASE_ $strategy" -- $copyname `;
          #create a run xml command for the new xml file
          $logfile = $strategy.".log";
          print RUNXMLS "../../pmpl -f $copyname > $logfile &\n";
        }
      print SEEDS "$seed\n";
      print RUNXMLS "\n";
}

