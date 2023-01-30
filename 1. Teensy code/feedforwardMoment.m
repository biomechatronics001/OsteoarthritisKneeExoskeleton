close all
clear
t=0:0.001:1;
M_slow=-[-0.187313465217391
-0.166743756521739
-0.125685434347826
-0.0961715686956522
-0.0872875613043478
-0.0710065086956522
-0.0386212808695652
-0.00326381565217391
0.0257202900000000
0.0507355491304348
0.0723358695652174
0.0901492608695652
0.104440103043478
0.115187985652174
0.122012579565217
0.125370507391304
0.125526926086957
0.123159578260870
0.119347969565217
0.114117665217391
0.107375013043478
0.0983948695652174
0.0868552043478261
0.0736207565217391
0.0601447521739131
0.0461947260869565
0.0320804217391304
0.0184521508695652
0.00498929508695653
-0.00937227534782608
-0.0249116230434783
-0.0414483565217391
-0.0582971360869565
-0.0755862260869565
-0.0930193565217391
-0.109871460869565
-0.126028878260870
-0.141404872608696
-0.155740366521739
-0.168394926086957
-0.179050060869565
-0.186980866652174
-0.191938536086957
-0.193342886956522
-0.190937721739130
-0.184566547826087
-0.174223927391304
-0.160186249260870
-0.142857637826087
-0.122875523043478
-0.101071694347826
-0.0781230391304348
-0.0551772782608696
-0.0328029391304348
-0.0100876100000000
0.0132891686956522
0.0358337347826087
0.0556957173913044
0.0706185695652174
0.0793385230434783
0.0795415330434783
0.0696229565217391
0.0578715086956522
0.0410385204347826
0.0214317686956522
0.00704787086956522
0.0112767552173913
0.0173569552173913
0.0286752159565217
0.0341370913043478
0.0330100043478261
0.0272044421739131
0.0193121578260870
0.0109707847826087
0.00279593556521739
-0.00464656360869565
-0.0114498821739130
-0.0179402453478261
-0.0240784695652174
-0.0296260230434783
-0.0345384434782609
-0.0387668869565217
-0.0427241130434783
-0.0465279826086957
-0.0498396608695652
-0.0522542565217391
-0.0546754000000000
-0.0588345478260870
-0.0652877913043478
-0.0736371652173913
-0.0839907956521739
-0.0961530260869565
-0.109212652173913
-0.122711408695652
-0.136174621739130
-0.147716630434783
-0.156907121739130
-0.167972473913043
-0.179322724782609
-0.169568938652174
-0.189475502434783];
M=-[-0.274134756521739
-0.243692673913043
-0.190619630434783
-0.122994126086957
-0.0756346091304348
-0.0347652782608696
0.0282774347826087
0.114748669565217
0.201773406956522
0.274852373913044
0.333380820869565
0.377808913043478
0.408050873913043
0.425573347826087
0.431471669565217
0.426427800000000
0.411711073913044
0.389469252173913
0.361826071130435
0.330461295652174
0.296943930434783
0.262577100000000
0.227947343478261
0.193178211304348
0.158588473913043
0.124773947826087
0.0919967652173913
0.0608821521739131
0.0317137260869565
0.00389105652173913
-0.0231369173913044
-0.0492890086956522
-0.0743652652173913
-0.0981079130434783
-0.120153156521739
-0.140660639130435
-0.160140008695652
-0.178565430434783
-0.194673400000000
-0.207891613043478
-0.217645665217391
-0.223337278260870
-0.224372891304348
-0.220307017391304
-0.210764108695652
-0.195679269565217
-0.175265260869565
-0.150259704347826
-0.121670286956522
-0.0906989652173913
-0.0584788186956522
-0.0258086956521739
0.00644687826086956
0.0365142956521739
0.0624673304347826
0.0838340301304348
0.100061542173913
0.110001956521739
0.112476130434783
0.104182600000000
0.0840706086956522
0.0567226130434783
0.0305628997826087
0.0244076556521739
0.0365809347826087
0.0536239665217391
0.0685220826086957
0.0741314478260870
0.0710693652173913
0.0622387695652174
0.0510111608695652
0.0400780130434783
0.0305750362173913
0.0227883900000000
0.0163785856521739
0.0103497721304348
0.00413345447826087
-0.00205718478260870
-0.00816947391304348
-0.0143929386956522
-0.0207693860869565
-0.0273025565217391
-0.0338767073913044
-0.0402447652173913
-0.0462542782608696
-0.0521501956521739
-0.0589653565217391
-0.0678964608695652
-0.0803662652173913
-0.0971012695652174
-0.117619265217391
-0.140745047826087
-0.165168926086957
-0.190814247826087
-0.218944217391304
-0.250232869565217
-0.281237608695652
-0.303515652173913
-0.304970000000000
-0.270198808695652
-0.285798082608696];
M=interp1(0:1/100:100/100,M,t);
M=[M M M M M];
[b,a]=butter(2,4/500);
M_filter=filtfilt(b,a,M);
figure
plot(M)
hold on
plot(M_filter)
moment=M_filter(2003:3003);
Moment=table(moment);
figure
plot(moment)

M_slow=interp1(0:1/100:100/100,M_slow,t);
M_slow=[M_slow M_slow M_slow M_slow M_slow];
[b,a]=butter(2,4/500);
M_slow_filter=filtfilt(b,a,M_slow);
figure
plot(M_slow)
hold on
plot(M_slow_filter)
moment_slow=M_slow_filter(2003:3003);
Moment_slow=table(moment_slow);
figure
plot(moment_slow)

kd=[0.8*ones(1,600) 0.4*ones(1,401)];
bd=[0.16*ones(1,600) 0.08*ones(1,401)];
kd=[kd kd kd kd kd];
bd=[bd bd bd bd bd];
[b,a]=butter(1,4/500);
kd=filtfilt(b,a,kd);
bd=filtfilt(b,a,bd);
Kd=table(kd(2003:3003));
Bd=table(bd(2003:3003));
figure
subplot(2,1,1);
plot(kd(2003:3003));
subplot(2,1,2);
plot(bd(2003:3003));

writetable(Moment);
writetable(Moment_slow);
writetable(Kd);
writetable(Bd);