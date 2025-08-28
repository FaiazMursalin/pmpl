#!/usr/bin/perl

#file path
$src_path = '/users/mghosh/pmpl/branches/src/EllipsoidTests/';

#list of strategies that were run (only put the unique part of the name)
@m_ARR = (
  "RapidFree",
  "RapidQuick",
  "PQPFree",
  "PQPQuick",
);

#if this is a new run, delete last output file
$resultsFile = 'compiledStats.txt';
if(-e $resultsFile) {
  `rm $resultsFile`;
}

open(STATFILE, ">>$resultsFile");

#loop through strategies
foreach $m (@m_ARR) {
  #print "m=".$m."\n";
  @flist=();

  #open the file: build full name
  open(LS ,"ls  ".$m."*.stat|");
  while (<LS> ){
    chomp;
    push @flist, $_;
  }
  close (LS);
  printf STATFILE "$m\n" ;

  #add or remove things from this list depending on what you want to extract
  printf STATFILE  ("%-55s %10s %10s %10s %10s %10s\n","seed","Collision", "Time", "Nodes", "Edges","CC1"); #Table heading for each m value
  printf STATFILE  "----------------------------------------------------------------------------------------------------------------------------\n";

  #variables
  $avgedge=0.0;
  $avgtime=0.0;
  $avgcol=0.0;
  $avgnod=0.0;
  $avgsuccess=0.0;
  $avgpercent1=0.0;
  $avgpercent2=0.0;
  $avgpercent3=0.0;
  $avgpercent4=0.0;
  $avgcon=0.0;
  $avgcon1=0.0;
  $count = 0.0;     #count the number of successful solvers

  #iterating over each seed file
  foreach $file (@flist) {
    $count++;
    $time=0.0;
    $nod=0.0;
    $percent1=0.0;  print "$file\n";

    #extracting nodes
    @nodlist=();
    open(NOD, "cat ".$file." | egrep \"Number of Nodes\"|colrm 1 17 |");
    while (<NOD> ){
      chomp;
      push @nodlist, $_;
    }
    close (NOD);
    $nod = $nodlist[0];
    $avgnod+= $nod;
    print "$nod\n";


    #extracting success rate
    @slist=();
    open(SUC, "cat ".$file." | egrep \"Success %\"|colrm 1 13 |");
    while (<SUC> ){
      chomp;
      push @slist, $_;
    }
    close (SUC);
    $suc = $slist[0];
    $avgsuccess+= $suc;
    print "$suc\n";

    #extracting collision Detection
    @clist=();
    open(COL, "cat ".$file." | egrep \"Total Cfg::isCollision\"|colrm 1 27 |");
    while (<COL> ){
      chomp;
      push @clist, $_;
    }
    close (COL);
    $col = $clist[0];
    $avgcol+= $col;
    print "$col\n";

    #analyzing connectivity based on connected components
    @cclist=();
    open(CON, "cat ".$file." | egrep \"There are\"|cut -c 11-13 |");
    while (<CON> ){
      chomp;
      push @cclist, $_;
    }
    close (CON);
    $con = $cclist[0];
    $avgcon+= $con;
    print "$con\n";

#extracting largest cc
    @ccclist=();
    open(CON1, "cat ".$file." | egrep \"0]:\" |cut -c 8-12|");
    while (<CON1> ){
      chomp;
      push @ccclist, $_;
    }
    close (CON1);
    $con1 = $ccclist[0];
    $avgcon1+= $con1;
    $percent1+= $con1/$nod * 100;
    $avgpercent1+=$percent1;

    print "$con1\n";
    print "$avgpercent\n";

#extracting second largest cc
    @cc2list=();
    open(CON2, "cat ".$file." | egrep \"2]:\" |cut -c 8-9|");
    while (<CON2> ){
      chomp;
      push @cc2list, $_;
    }
    close (CON2);
    $con2 = $cc2list[0];
    $avgcon2+= $con2;
    $avgpercent2+= $con2/$nod * 100;
    print "$con2\n";

    #extracting 3rd largest cc
    @cc3list=();
    open(CON3, "cat ".$file." | egrep \"3]:\" |cut -c 8-9|");
    while (<CON3> ){
      chomp;
      push @cc3list, $_;
    }
    close (CON3);
    $con3 = $cc3list[0];
    $avgcon3+= $con3;
    $avgpercent3+= $con3/$nod * 100;
    print "$con3\n";

    #extracting number of edges
    @nlist=();
    open(NODE, "cat ".$file." | egrep \"Number of Edges\" |colrm 1 17 |");
    while (<NODE> ){
      chomp;
      push @nlist, $_;
    }
    close (NODE);
    $edge = $nlist[0];
    $avgedge+=$edge;
    print "$edge\n";

    #extracting Time
    @tlist=();
    open(TIME, "cat ".$file." | egrep \"Total Node Generation\" |colrm 1 73 |");
    while (<TIME> ){
      chomp;
      push @tlist, $_;
    }
    close (TIME);
    $time = $tlist[0];
    $avgtime+= $time;
    print "$time\n";
    printf STATFILE ("%-55s %10d %10.2f %10.2f %10.2f %10.2f\n",$file,$col,$time,$nod,$edge, $percent1);


  }
  $percentcc= $avgnod/12;
  printf STATFILE  "----------------------------------------------------------------------------------------------------------------------------\n";

  #print average values
  printf STATFILE ("%65d %10.2f %10.2f %10.2f %10.2f\n",
    $avgcol/$count,$avgtime/$count,$avgnod/$count,$avgedge/$count,$avgpercent1/$count);
  print STATFILE ("\n============================================================================================================================\n\n\n");
}

close(STATFILE);

