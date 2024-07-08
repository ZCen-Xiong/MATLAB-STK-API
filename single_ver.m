% show the rendezvous stage of the dragon ship to the ISS
% including propagate and access components

clc; clear; close all; warning('off');
% read STK scenario  path
addpath('.\STK_Scen');
% read MATLAB result  path
addpath('result2');
% load STK scenario 
% -------------------------  scenario load  --------------------
app = actxserver('STK11.application');
root = app.Personality2;
root.LoadScenario([pwd , '.\STK_Scen\intercept.sc']);
scn = root.CurrentScenario;
%% ----------------------- auto verify scripts -------------------------
total_index = 1; %  data  quantity 
% save constraints satisify  case  
A_result_data = zeros(total_index,4);

% save constraints result  'data index ',' lighting  sections ',' remote detectable  sections 
A_cell_constraints = cell(total_index+1,3);
A_cell_constraints{1,1} = 'data index ';
A_cell_constraints{1,2} = ' lighting  sections ';
A_cell_constraints{1,3} = ' remote detectable  sections ';

global_index = 10
filename = ['data',num2str(global_index),'.mat'];
fprintf(filename);
load(filename);

%  load dynamics
%% twobody
% manuver_data = data.twobody;
% dynamics = 'Earth Point Mass';

%% J2
manuver_data = data.J2;
dynamics = 'Earth J2';

%% Perturbation 
% manuver_data = data.perturb;
% dynamics = 'Earth High for itc';


imp_num = size(data.twobody,1);


dryMass = 1e3; %kg
dragArea = 1; %m
SRPArea = 28.5;
cd = 2.4;
cr = 1.5;
ck = 0;
gps_k1 = 0;
gps_k2 = 0;

% dynamics = 'Earth HPOP Default v10';

%  load frame
global_frame = 'CentralBody/Earth ICRF';

try
    name = 'DRG' ;
    sat = scn.Children.Item(name);
    sat.Unload;%delete  spacecrafts 
    name = 'ISS' ;
    sat = scn.Children.Item(name);
    sat.Unload;%delete  spacecrafts 
end
Dragon = scn.Children.New(18,'DRG');
ISS = scn.Children.New(18,'ISS');
Dragon.SetPropagatorType('ePropagatorAstrogator');
ISS.SetPropagatorType('ePropagatorAstrogator');
Dragon.VO.Model.DetailThreshold.All = 1e12;
Dragon.VO.Model.DetailThreshold.MarkerLabel = 0.001;


% read  spacecrafts  initial 历元
date = data.date0;
t0 = datetime(date(1),date(2),date(3),date(4),date(5),date(6));
t0_sec = datevec(t0);

%  add DRG initial  state 
% 注：在STK中 add  initial  state 和 orbital 递推
%% --------------------  add Init --------------------
% 抓取MCS
DRG_Mcs = Dragon.Propagator.MainSequence;
ISS_Mcs = ISS.Propagator.MainSequence;
% 清空MCS
DRG_Mcs.RemoveAll;
ISS_Mcs.RemoveAll;
%  add init sec
init_DRG = DRG_Mcs.Insert('eVASegmentTypeInitialState' , 'init' , '-');
init_ISS = ISS_Mcs.Insert('eVASegmentTypeInitialState' , 'init' , '-');

% define frame
init_DRG.CoordSystemName = global_frame;
init_ISS.CoordSystemName = global_frame;
% define  state  type
init_DRG.SetElementType('eVAElementTypeKeplerian');
init_ISS.SetElementType('eVAElementTypeKeplerian');
%  orbital initial value
% 防奇异偏置
bias_oe = [0,0,0,0,0,0];
% p_oe = data.DRG0 + bias_oe;
% t_oe = data.ISS0 + bias_oe;
p_oe = data.DRG0 ;
t_oe = data.ISS0 ;
init_DRG.OrbitEpoch = datestr(t0 , 'dd mmm yyyy HH:MM:SS.FFF');
init_DRG.SpacecraftParameters.DryMass = dryMass;
init_DRG.SpacecraftParameters.DragArea = dragArea;
init_DRG.SpacecraftParameters.Cd = cd;
init_DRG.SpacecraftParameters.Cr = cr;
init_DRG.SpacecraftParameters.CK = ck;
init_DRG.SpacecraftParameters.SolarRadiationPressureArea = SRPArea;
init_DRG.SpacecraftParameters.K1 = gps_k1;
init_DRG.SpacecraftParameters.K2 = gps_k2;

init_DRG.Element.ElementType = 'eVAElementOsculating';
init_DRG.Element.SemiMajorAxis = p_oe(1)*1e-3;
init_DRG.Element.Eccentricity = p_oe(2);
init_DRG.Element.Inclination = p_oe(3)*180/pi;
init_DRG.Element.RAAN = p_oe(4)*180/pi;
init_DRG.Element.ArgOfPeriapsis = p_oe(5)*180/pi;
init_DRG.Element.TrueAnomaly = p_oe(6)*180/pi;

%
init_ISS.OrbitEpoch = datestr(t0 , 'dd mmm yyyy HH:MM:SS.FFF');
init_ISS.SpacecraftParameters.DryMass = dryMass;
init_ISS.SpacecraftParameters.DragArea = dragArea;
init_ISS.SpacecraftParameters.Cd = cd;
init_ISS.SpacecraftParameters.Cr = cr;
init_ISS.SpacecraftParameters.CK = ck;
init_ISS.SpacecraftParameters.SolarRadiationPressureArea = SRPArea;
init_ISS.SpacecraftParameters.K1 = gps_k1;
init_ISS.SpacecraftParameters.K2 = gps_k2;
init_ISS.Element.ElementType = 'eVAElementOsculating';
init_ISS.Element.SemiMajorAxis = t_oe(1)*1e-3;
init_ISS.Element.Eccentricity = t_oe(2);
init_ISS.Element.Inclination = t_oe(3)*180/pi;
init_ISS.Element.RAAN = t_oe(4)*180/pi;
init_ISS.Element.ArgOfPeriapsis = t_oe(5)*180/pi;
init_ISS.Element.TrueAnomaly = t_oe(6)*180/pi;

%% --------------------  add Dragon maneuver  --------------------
%  add Prop sec
PlanT = manuver_data(:,1);
PlanDV = manuver_data(:,2:4)*1e-3;
PlanT = [0;PlanT];

for j = 1:imp_num
    propagateName = ['Prop',num2str(j)];
    DRG_Mcs.Insert('eVASegmentTypePropagate' ,propagateName , '-');
    DRG_Mcs.Item(propagateName).Properties.Color = 60000;
    DRG_Mcs.Item(propagateName).PropagatorName = dynamics;
    DRG_Mcs.Item(propagateName).StoppingConditions.Item('Duration').Properties.Trip = PlanT(j+1) - PlanT(j);
    
    %  impulse 1
    maneuverName = ['Maneuver',num2str(j)];
    DRG_Mcs.Insert('eVASegmentTypeManeuver',maneuverName,'-');
    DRG_Mcs.Item(maneuverName).Maneuver.SetAttitudeControlType('eVAAttitudeControlThrustVector');
    DRG_Mcs.Item(maneuverName).Maneuver.AttitudeControl.ThrustAxesName = global_frame;
    DRG_Mcs.Item(maneuverName).Maneuver.AttitudeControl.DeltaVVector.AssignCartesian(PlanDV(j,1),PlanDV(j,2),PlanDV(j,3));
end
%  run MCS
Dragon.Propagator.RunMCS;
DRG_final = DRG_Mcs.Item(['Prop',num2str(imp_num)]).FinalState.Element;
% Dragon ship  ending inertial velocity
rpf = [DRG_final.X,DRG_final.Y,DRG_final.Z];
vpf = [DRG_final.Vx,DRG_final.Vy,DRG_final.Vz];

%% --------------------  add  ISSet Propagation --------------------
%  add Prop sec
propagateName = ['Prop',num2str(1)];
ISS_Mcs.Insert('eVASegmentTypePropagate' ,propagateName , '-');
ISS_Mcs.Item(propagateName).Properties.Color = 100000;
ISS_Mcs.Item(propagateName).PropagatorName = dynamics;
ISS_Mcs.Item(propagateName).StoppingConditions.Item('Duration').Properties.Trip = PlanT(imp_num+1) - PlanT(1);
%  run MCS
ISS.Propagator.RunMCS;
ISS_final = ISS_Mcs.Item(propagateName).FinalState.Element;
% Int'l Space Station  ending inertial velocity
rtf = [ISS_final.X,ISS_final.Y,ISS_final.Z];
vtf = [ISS_final.Vx,ISS_final.Vy,ISS_final.Vz];
%%  build  celetial  observation 
% read  celetial  observation 信息
spst_oe = data.spst0;
spst_range = data.sprang;
%
spstandaccessinfo = cell(size(spst_oe,1)+1,4);
spstandaccessinfo{1,1} = ' name ';
spstandaccessinfo{1,2} = ' positions ';
spstandaccessinfo{1,3} = '可见次数';
spstandaccessinfo{1,4} ='可见弧 sec起止点';
%
for i = 1:size(spst_oe,1) % line by line read 
    name = 'spst_' ;
    try
        sat = scn.Children.Item([name,num2str(i)]);%识别潜在重名 spacecrafts 
        sat.Unload;%delete  spacecrafts 
    end
    spstname = ['spst_',num2str(i)];
    spst = scn.Children.New(18,spstname);
    spst.SetPropagatorType('ePropagatorAstrogator');
    spst_Mcs = spst.Propagator.MainSequence;
    spst_Mcs.RemoveAll;
    %  add init sec
    init_spst = spst_Mcs.Insert('eVASegmentTypeInitialState' , 'init' , '-');
    % define frame
    init_spst.CoordSystemName = 'CentralBody/Earth J2000';
    % define  state  type
    init_spst.SetElementType('eVAElementTypeKeplerian');
    % initial value
    init_spst.OrbitEpoch = datestr(t0 , 'dd mmm yyyy HH:MM:SS.FFF');
    init_spst.Element.ElementType = 'eVAElementOsculating';
    init_spst.Element.SemiMajorAxis = spst_oe(i,1)*1e-3;
    init_spst.Element.Eccentricity = spst_oe(i,2);
    init_spst.Element.Inclination = spst_oe(i,3)*180/pi;
    init_spst.Element.RAAN = spst_oe(i,4)*180/pi;
    init_spst.Element.ArgOfPeriapsis = spst_oe(i,5)*180/pi;
    init_spst.Element.TrueAnomaly = spst_oe(i,6)*180/pi;
    propagateName = ['Prop',num2str(1)];
    spst_Mcs.Insert('eVASegmentTypePropagate' ,propagateName , '-');
    spst_Mcs.Item(propagateName).Properties.Color = 70000;
    spst_Mcs.Item(propagateName).PropagatorName = 'Earth Point Mass';
    spst_Mcs.Item(propagateName).StoppingConditions.Item('Duration').Properties.Trip = PlanT(imp_num+1) - PlanT(1);
    
    spst.Propagator.RunMCS;
    sensorname = ['space_sen_',num2str(i)];
    spst_sen = spst.Children.New('eSensor',sensorname);
    spst_sen.CommonTasks.SetPatternSimpleConic(spst_range(i,1)*180/pi,1);
    %获取 ground  observation  name 及纬度经度高度，并写入到cell中
    spstandaccessinfo{i+1,1} = spst.InstanceName;
    spstandaccessinfo{i+1,2} = spst_oe(i,:);
    %计算可见性，获取读 spacecrafts 可见次数、每个可见弧 sec的起止时间、弧 sec时长
    %并写入到cell中，以便最后输出
    spst2sataccess = spst_sen.GetAccessToObject(Dragon);
    spst2sataccess.ComputeAccess();
    spstandaccessinfo{i+1,3} = spst2sataccess.ComputedAccessIntervalTimes.Count;
    %获取
    access_st_end = cell(spst2sataccess.ComputedAccessIntervalTimes.Count,1);
    for j = 1:spst2sataccess.ComputedAccessIntervalTimes.Count
        [str,sto] = spst2sataccess.ComputedAccessIntervalTimes.GetInterval(j-1);
        spst2sataccess. AccessTimePeriod = 'eUserSpecAccessTime';
        spst2sataccess.SpecifyAccessTimePeriod(str,sto)
        accessduration = spst2sataccess.AccessTimePeriodData.Duration;
        access_st_end{j}= {str,sto,accessduration};
    end
    spstandaccessinfo{i+1,4} = access_st_end;
    spst2sataccess.RemoveAccess();
end




% read  ground  observation 信息
gst_loc = data.gst0;
gst_range = data.grang;
%%  build  ground  observation 
gstandaccessinfo = cell(2,4);% 多个时，第一个元素改为，size(gst_loc,1)+1
gstandaccessinfo{1,1} = ' name ';
gstandaccessinfo{1,2} = ' positions ';
gstandaccessinfo{1,3} = '可见次数';
gstandaccessinfo{1,4} ='可见弧 sec起止点';

for i = 1:1  % line by line read  all  scenario 中只有一个 ground  observation ,多个改为，size(gst_loc,1)
    name = 'gst_' ;
    try
        gst_pre = scn.Children.Item([name,num2str(i)]);%识别潜在重名 spacecrafts 
        gst_pre.Unload;%delete  spacecrafts 
    end
    gstname = ['gst_',num2str(i)];
    gst = scn.Children.New('eFacility',gstname);
    % 修一个bug
    if gst_loc(i,1)== 20
        gst_loc(i,1) = 20 - 6*pi;
    end
    gst.Position.AssignGeodetic(gst_loc(i,1)*180/pi,gst_loc(i,2)*180/pi,gst_loc(i,3)*1e-3);
    %设置 ground  observation constraints
    gstConstraints = gst.AccessConstraints;
    %设置最大距离constraints
    rangeCstr = gstConstraints.AddConstraint('eCstrRange');
    rangeCstr.EnableMax = 1;
    rangeCstr.Max = gst_range(i,2)*1e-3;
    %设置仰角constraints
    elevationCstr = gstConstraints.AddConstraint('eCstrElevationAngle');
    elevationCstr.EnableMin = 1;
    elevationCstr.Min = gst_range(i,1)*180/pi;
    %获取 ground  observation  name 及纬度经度高度，并写入到cell中
    gstandaccessinfo{i+1,1} = gst.InstanceName;
    gstandaccessinfo{i+1,2} = gst.Position.QueryPlanetodeticArray;
    %计算可见性，获取读 spacecrafts 可见次数、每个可见弧 sec的起止时间、弧 sec时长
    %并写入到cell中，以便最后输出
    gst2sataccess = gst.GetAccessToObject(Dragon);
    gst2sataccess.ComputeAccess();
    gstandaccessinfo{i+1,3} = gst2sataccess.ComputedAccessIntervalTimes.Count;
    %获取
    access_st_end = cell(gst2sataccess.ComputedAccessIntervalTimes.Count,1);
    for j = 1:gst2sataccess.ComputedAccessIntervalTimes.Count
        [str,sto] = gst2sataccess.ComputedAccessIntervalTimes.GetInterval(j-1);
        gst2sataccess. AccessTimePeriod = 'eUserSpecAccessTime';
        gst2sataccess.SpecifyAccessTimePeriod(str,sto)
        accessduration = gst2sataccess.AccessTimePeriodData.Duration;
        access_st_end{j}= {str,sto,accessduration};
    end
    gstandaccessinfo{i+1,4} = access_st_end;
    gst2sataccess.RemoveAccess();
end


%% -------------------- % 距离
% 两星连线 p指向t
Dragon2ISSaccess = Dragon.GetAccessToObject(ISS);
Dragon2ISSaccess.ComputeAccess()
% P2T_AER = Dragon2ISSaccess.DataProviders.Item('Points(ICRF)').Group.Item('ToObject').Exec(scn.StartTime,scn.StopTime,120);
% P2T_RightA = cell2mat(P2T_AER.DataSets.GetDataSetByName('RightAscension').GetValues);
% P2T_Decl = cell2mat(P2T_AER.DataSets.GetDataSetByName('Declination').GetValues);
% P2T_ran = cell2mat(P2T_AER.DataSets.GetDataSetByName('Magnitude').GetValues);
% P2T_time = cell2mat(P2T_AER.DataSets.GetDataSetByName('Time').GetValues);
% 对于中途有被地球阻挡的情况，AER不可使用
% P2T_AER = Dragon2ISSaccess.DataProviders.Item('AER Data').Group.Item('VVLH CBF').Exec(str,sto,120);
% P2T_azi = cell2mat(P2T_AER.DataSets.GetDataSetByName('Azimuth').GetValues);
% P2T_ele = cell2mat(P2T_AER.DataSets.GetDataSetByName('Elevation').GetValues);
% P2T_ran = cell2mat(P2T_AER.DataSets.GetDataSetByName('Range').GetValues);
% P2T_time = cell2mat(P2T_AER.DataSets.GetDataSetByName('Time').GetValues);



%% -------------------- %  lighting constraints
% 太阳连线 p指向sun
name = 'Sun' ;
sun = scn.Children.Item(name);
Dragon2sun_access = Dragon.GetAccessToObject(sun);
Dragon2sun_access.ComputeAccess()
% 获取访问的间隔数
DRG_sun_accessinfo = cell(1,2);
DRG_sun_accessinfo{1,1} = 'AER';
DRG_sun_accessinfo{1,2} = '可见弧 sec起止点';
% 弧 secAER
access_aer = cell(Dragon2sun_access.ComputedAccessIntervalTimes.Count,1);
% 弧 sec起止
access_st_end = cell(Dragon2sun_access.ComputedAccessIntervalTimes.Count,1);
for j = 1:Dragon2sun_access.ComputedAccessIntervalTimes.Count
    [str,sto] = Dragon2sun_access.ComputedAccessIntervalTimes.GetInterval(j-1);
    Dragon2sun_access. AccessTimePeriod = 'eUserSpecAccessTime';
    Dragon2sun_access.SpecifyAccessTimePeriod(str,sto)
    %     P2S_AER = Dragon2sun_access.DataProviders.Item('AER Data').Group.Item('VVLH CBF').Exec(str,sto,120);
    %     P2S_azi = cell2mat(P2S_AER.DataSets.GetDataSetByName('Azimuth').GetValues);
    %     P2S_ele = cell2mat(P2S_AER.DataSets.GetDataSetByName('Elevation').GetValues);
    %     P2S_ran = cell2mat(P2S_AER.DataSets.GetDataSetByName('Range').GetValues);
    %     P2S_time = cell2mat(P2S_AER.DataSets.GetDataSetByName('Time').GetValues);
    % 这里读的已经是RA和DC了，不是AER，只是为了统一符号
    P2S_AER = Dragon2sun_access.DataProviders.Item('Points(ICRF)').Group.Item('ToObject').Exec(str,sto,120);
    P2S_azi = cell2mat(P2S_AER.DataSets.GetDataSetByName('RightAscension').GetValues);
    P2S_ele = cell2mat(P2S_AER.DataSets.GetDataSetByName('Declination').GetValues);
    P2S_time = cell2mat(P2S_AER.DataSets.GetDataSetByName('Time').GetValues);
    access_aer{j}= {P2S_time,P2S_azi,P2S_ele};
    accessduration = Dragon2sun_access.AccessTimePeriodData.Duration;
    access_st_end{j}= {str,sto,accessduration};
end
% 太阳角
DRG_sun_accessinfo{2,1} = access_aer;
% 太阳时间
DRG_sun_accessinfo{2,2} = access_st_end;
%% —————————————————— 命中前的太阳角
Dragon2sun_access.RemoveAccess();
sun_angle_end = [access_aer{end}{2}(end),access_aer{end}{3}(end)];
sun_vec =  an2vec(sun_angle_end(1),sun_angle_end(2));
sat_vec = vtf - vpf;
fprintf('接近太阳角/deg')
sun_vec_angle = acos(dot(sat_vec,sun_vec)/(norm(sat_vec)*norm(sun_vec)))*180/pi
fprintf('终端距离/km')
dist_final = norm(rpf-rtf)
%% ----------------------------------  lighting  ----------------------------------
sun_period = zeros(1,2); % 至少有1行
sun_c_i = 1;
for sun_i = 2:2  % 第一行是 data  name , go through 三个 observation 
    for time_i = 1:size(DRG_sun_accessinfo{sun_i,2},1) %  go through 某个 observation all 弧 sec
        timesuneriod = DRG_sun_accessinfo{sun_i, 2}{time_i, 1};
        dateTime1 = datevec(datetime(timesuneriod{1}, 'InputFormat', 'dd MMM yyyy HH:mm:ss.SSS' ,'Locale', 'en_US')); % 从char 到date 到s
        dateTime2 = datevec(datetime(timesuneriod{2}, 'InputFormat', 'dd MMM yyyy HH:mm:ss.SSS' ,'Locale', 'en_US'));
        sun_period(sun_c_i,:) = [etime(dateTime1, t0_sec),etime(dateTime2, t0_sec)];
        sun_c_i = sun_c_i+1;
    end
end
%  lighting 合集
fprintf(' lighting 总时间 sec：');
sun_union = interval_union(sun_period)

%% ----------------------------------  remote detectable  ----------------------------------

%  remote detectable constraints
% read  celetial 
sp_period = zeros(size(spst_oe,1),2); % 至少有size(spst_oe,1)行
sp_c_i = 1;
for sp_i = 2: size(spst_oe,1)+1  % 第一行是 data  name , go through 三个 observation 
    for time_i = 1:size(spstandaccessinfo{sp_i,4},1) %  go through 某个 observation all 弧 sec
        timesperiod = spstandaccessinfo{sp_i, 4}{time_i, 1};
        dateTime1 = datevec(datetime(timesperiod{1}, 'InputFormat', 'dd MMM yyyy HH:mm:ss.SSS' ,'Locale', 'en_US')); % 从char 到date 到s
        dateTime2 = datevec(datetime(timesperiod{2}, 'InputFormat', 'dd MMM yyyy HH:mm:ss.SSS' ,'Locale', 'en_US'));
        sp_period(sp_c_i,:) = [etime(dateTime1, t0_sec),etime(dateTime2, t0_sec)];
        sp_c_i = sp_c_i+1;
    end
end

% read 地基
gr_period = zeros(1,2); % 至少有1行
gr_c_i = 1;
for sp_i = 2:2  % line by line read  all  scenario 中只有一个 ground  observation ,多个改为，size(loc,1)
    for time_i = 1:size(gstandaccessinfo{sp_i,4},1) %  go through 某个 observation all 弧 sec
        timesperiod = gstandaccessinfo{sp_i, 4}{time_i, 1};
        dateTime1 = datevec(datetime(timesperiod{1}, 'InputFormat', 'dd MMM yyyy HH:mm:ss.SSS' ,'Locale', 'en_US')); % 从char 到date 到s
        dateTime2 = datevec(datetime(timesperiod{2}, 'InputFormat', 'dd MMM yyyy HH:mm:ss.SSS' ,'Locale', 'en_US'));
        gr_period(gr_c_i,:) = [etime(dateTime1, t0_sec),etime(dateTime2, t0_sec)];
        gr_c_i = gr_c_i+1;
    end
end

%  remote detectable 合集
total_period = [sp_period;gr_period];
fprintf(' remote detectable 总时间 sec：');
commu_union = interval_union(total_period)
in_sun = 0;
in_commu = 0;
for dv_i = 1:imp_num
    dv_time = PlanT(dv_i);
    in_sun = in_sun + any(dv_time >= sun_union(:, 1) & dv_time <= sun_union(:, 2));
    in_commu = in_commu + any(dv_time >= commu_union(:, 1) & dv_time <= commu_union(:, 2));
end

if in_sun == imp_num
    sun_index = 1;
else
    sun_index = 0;
end

if in_commu == imp_num
    commu_index = 1;
else
    commu_index = 0;
end
A_cell_constraints{global_index + 1,1} = global_index;
A_cell_constraints{global_index + 1,2} = sun_union;
A_cell_constraints{global_index + 1,3} = commu_union;
A_result_data(global_index,1) = sun_vec_angle;
A_result_data(global_index,2) = dist_final;
A_result_data(global_index,3) = sun_index;
A_result_data(global_index,4) = commu_index;

