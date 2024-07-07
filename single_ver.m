clc; clear; close all; warning('off');
% ��ȡSTK�����ļ���
addpath('.\STK_Scen');
% ��ȡMATLAB����ļ���
addpath('result2');
% ����STK����
% ------------------------- �������� --------------------
app = actxserver('STK11.application');
root = app.Personality2;
root.LoadScenario([pwd , '.\STK_Scen\intercept.sc']);
scn = root.CurrentScenario;
%% -----------------------�Զ����ű� -------------------------
total_index = 1; % ��������
% ���Լ���������  �ӽ���   ����    �Ƿ��������  �Ƿ������
A_result_data = zeros(total_index,4);

% ���Լ����� 'data���','��������','�������
A_cell_constraints = cell(total_index+1,3);
A_cell_constraints{1,1} = 'data���';
A_cell_constraints{1,2} = '��������';
A_cell_constraints{1,3} = '�������';

global_index = 10
filename = ['data',num2str(global_index),'.mat'];
fprintf(filename);
load(filename);

% ���ض���ѧ
%% ����
% manuver_data = data.twobody;
% dynamics = 'Earth Point Mass';

%% J2
manuver_data = data.J2;
dynamics = 'Earth J2';

%% �߾���
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

% ��������ϵ
global_frame = 'CentralBody/Earth ICRF';

try
    name = 'pur' ;
    sat = scn.Children.Item(name);
    sat.Unload;%ɾ������
    name = 'targ' ;
    sat = scn.Children.Item(name);
    sat.Unload;%ɾ������
end
pursue = scn.Children.New(18,'pur');
targ = scn.Children.New(18,'targ');
pursue.SetPropagatorType('ePropagatorAstrogator');
targ.SetPropagatorType('ePropagatorAstrogator');
pursue.VO.Model.DetailThreshold.All = 1e12;
pursue.VO.Model.DetailThreshold.MarkerLabel = 0.001;


% ��ȡ���ǳ�ʼ��Ԫ
date = data.date0;
t0 = datetime(date(1),date(2),date(3),date(4),date(5),date(6));
t0_sec = datevec(t0);

% ���pur��ʼ״̬
% ע����STK����ӳ�ʼ״̬�͹������
%% -------------------- ���Init --------------------
% ץȡMCS
pur_Mcs = pursue.Propagator.MainSequence;
targ_Mcs = targ.Propagator.MainSequence;
% ���MCS
pur_Mcs.RemoveAll;
targ_Mcs.RemoveAll;
% ���init��
init_pur = pur_Mcs.Insert('eVASegmentTypeInitialState' , 'init' , '-');
init_targ = targ_Mcs.Insert('eVASegmentTypeInitialState' , 'init' , '-');

% ��������ϵ
init_pur.CoordSystemName = global_frame;
init_targ.CoordSystemName = global_frame;
% ����״̬����
init_pur.SetElementType('eVAElementTypeKeplerian');
init_targ.SetElementType('eVAElementTypeKeplerian');
% �����ֵ
% ������ƫ��
bias_oe = [0,0,0,0,0,0];
% p_oe = data.pur0 + bias_oe;
% t_oe = data.targ0 + bias_oe;
p_oe = data.pur0 ;
t_oe = data.targ0 ;
init_pur.OrbitEpoch = datestr(t0 , 'dd mmm yyyy HH:MM:SS.FFF');
init_pur.SpacecraftParameters.DryMass = dryMass;
init_pur.SpacecraftParameters.DragArea = dragArea;
init_pur.SpacecraftParameters.Cd = cd;
init_pur.SpacecraftParameters.Cr = cr;
init_pur.SpacecraftParameters.CK = ck;
init_pur.SpacecraftParameters.SolarRadiationPressureArea = SRPArea;
init_pur.SpacecraftParameters.K1 = gps_k1;
init_pur.SpacecraftParameters.K2 = gps_k2;

init_pur.Element.ElementType = 'eVAElementOsculating';
init_pur.Element.SemiMajorAxis = p_oe(1)*1e-3;
init_pur.Element.Eccentricity = p_oe(2);
init_pur.Element.Inclination = p_oe(3)*180/pi;
init_pur.Element.RAAN = p_oe(4)*180/pi;
init_pur.Element.ArgOfPeriapsis = p_oe(5)*180/pi;
init_pur.Element.TrueAnomaly = p_oe(6)*180/pi;

%
init_targ.OrbitEpoch = datestr(t0 , 'dd mmm yyyy HH:MM:SS.FFF');
init_targ.SpacecraftParameters.DryMass = dryMass;
init_targ.SpacecraftParameters.DragArea = dragArea;
init_targ.SpacecraftParameters.Cd = cd;
init_targ.SpacecraftParameters.Cr = cr;
init_targ.SpacecraftParameters.CK = ck;
init_targ.SpacecraftParameters.SolarRadiationPressureArea = SRPArea;
init_targ.SpacecraftParameters.K1 = gps_k1;
init_targ.SpacecraftParameters.K2 = gps_k2;
init_targ.Element.ElementType = 'eVAElementOsculating';
init_targ.Element.SemiMajorAxis = t_oe(1)*1e-3;
init_targ.Element.Eccentricity = t_oe(2);
init_targ.Element.Inclination = t_oe(3)*180/pi;
init_targ.Element.RAAN = t_oe(4)*180/pi;
init_targ.Element.ArgOfPeriapsis = t_oe(5)*180/pi;
init_targ.Element.TrueAnomaly = t_oe(6)*180/pi;

%% -------------------- ���Pursue ���� --------------------
% ���Prop��
PlanT = manuver_data(:,1);
PlanDV = manuver_data(:,2:4)*1e-3;
PlanT = [0;PlanT];

for j = 1:imp_num
    propagateName = ['Prop',num2str(j)];
    pur_Mcs.Insert('eVASegmentTypePropagate' ,propagateName , '-');
    pur_Mcs.Item(propagateName).Properties.Color = 60000;
    pur_Mcs.Item(propagateName).PropagatorName = dynamics;
    pur_Mcs.Item(propagateName).StoppingConditions.Item('Duration').Properties.Trip = PlanT(j+1) - PlanT(j);
    
    % ���� 1
    maneuverName = ['Maneuver',num2str(j)];
    pur_Mcs.Insert('eVASegmentTypeManeuver',maneuverName,'-');
    pur_Mcs.Item(maneuverName).Maneuver.SetAttitudeControlType('eVAAttitudeControlThrustVector');
    pur_Mcs.Item(maneuverName).Maneuver.AttitudeControl.ThrustAxesName = global_frame;
    pur_Mcs.Item(maneuverName).Maneuver.AttitudeControl.DeltaVVector.AssignCartesian(PlanDV(j,1),PlanDV(j,2),PlanDV(j,3));
end
% ����MCS
pursue.Propagator.RunMCS;
pur_final = pur_Mcs.Item(['Prop',num2str(imp_num)]).FinalState.Element;
% ׷���� ���������ٶ�
rpf = [pur_final.X,pur_final.Y,pur_final.Z];
vpf = [pur_final.Vx,pur_final.Vy,pur_final.Vz];

%% -------------------- ��� target Propagation --------------------
% ���Prop��
propagateName = ['Prop',num2str(1)];
targ_Mcs.Insert('eVASegmentTypePropagate' ,propagateName , '-');
targ_Mcs.Item(propagateName).Properties.Color = 100000;
targ_Mcs.Item(propagateName).PropagatorName = dynamics;
targ_Mcs.Item(propagateName).StoppingConditions.Item('Duration').Properties.Trip = PlanT(imp_num+1) - PlanT(1);
% ����MCS
targ.Propagator.RunMCS;
targ_final = targ_Mcs.Item(propagateName).FinalState.Element;
% Ŀ���� ���������ٶ�
rtf = [targ_final.X,targ_final.Y,targ_final.Z];
vtf = [targ_final.Vx,targ_final.Vy,targ_final.Vz];
%% �������վ
% ��ȡ���վ��Ϣ
spst_oe = data.spst0;
spst_range = data.sprang;
%
spstandaccessinfo = cell(size(spst_oe,1)+1,4);
spstandaccessinfo{1,1} = '����';
spstandaccessinfo{1,2} = 'λ��';
spstandaccessinfo{1,3} = '�ɼ�����';
spstandaccessinfo{1,4} ='�ɼ�������ֹ��';
%
for i = 1:size(spst_oe,1) % ���ж�ȡ
    name = 'spst_' ;
    try
        sat = scn.Children.Item([name,num2str(i)]);%ʶ��Ǳ����������
        sat.Unload;%ɾ������
    end
    spstname = ['spst_',num2str(i)];
    spst = scn.Children.New(18,spstname);
    spst.SetPropagatorType('ePropagatorAstrogator');
    spst_Mcs = spst.Propagator.MainSequence;
    spst_Mcs.RemoveAll;
    % ���init��
    init_spst = spst_Mcs.Insert('eVASegmentTypeInitialState' , 'init' , '-');
    % ��������ϵ
    init_spst.CoordSystemName = 'CentralBody/Earth J2000';
    % ����״̬����
    init_spst.SetElementType('eVAElementTypeKeplerian');
    % ��ֵ
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
    %��ȡ����վ���Ƽ�γ�Ⱦ��ȸ߶ȣ���д�뵽cell��
    spstandaccessinfo{i+1,1} = spst.InstanceName;
    spstandaccessinfo{i+1,2} = spst_oe(i,:);
    %����ɼ��ԣ���ȡ�����ǿɼ�������ÿ���ɼ����ε���ֹʱ�䡢����ʱ��
    %��д�뵽cell�У��Ա�������
    spst2sataccess = spst_sen.GetAccessToObject(pursue);
    spst2sataccess.ComputeAccess();
    spstandaccessinfo{i+1,3} = spst2sataccess.ComputedAccessIntervalTimes.Count;
    %��ȡ
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




% ��ȡ����վ��Ϣ
gst_loc = data.gst0;
gst_range = data.grang;
%% ��������վ
gstandaccessinfo = cell(2,4);% ���ʱ����һ��Ԫ�ظ�Ϊ��size(gst_loc,1)+1
gstandaccessinfo{1,1} = '����';
gstandaccessinfo{1,2} = 'λ��';
gstandaccessinfo{1,3} = '�ɼ�����';
gstandaccessinfo{1,4} ='�ɼ�������ֹ��';

for i = 1:1  % ���ж�ȡ ���г�����ֻ��һ������վ,�����Ϊ��size(gst_loc,1)
    name = 'gst_' ;
    try
        gst_pre = scn.Children.Item([name,num2str(i)]);%ʶ��Ǳ����������
        gst_pre.Unload;%ɾ������
    end
    gstname = ['gst_',num2str(i)];
    gst = scn.Children.New('eFacility',gstname);
    % ��һ��bug
    if gst_loc(i,1)== 20
        gst_loc(i,1) = 20 - 6*pi;
    end
    gst.Position.AssignGeodetic(gst_loc(i,1)*180/pi,gst_loc(i,2)*180/pi,gst_loc(i,3)*1e-3);
    %���õ���վԼ��
    gstConstraints = gst.AccessConstraints;
    %����������Լ��
    rangeCstr = gstConstraints.AddConstraint('eCstrRange');
    rangeCstr.EnableMax = 1;
    rangeCstr.Max = gst_range(i,2)*1e-3;
    %��������Լ��
    elevationCstr = gstConstraints.AddConstraint('eCstrElevationAngle');
    elevationCstr.EnableMin = 1;
    elevationCstr.Min = gst_range(i,1)*180/pi;
    %��ȡ����վ���Ƽ�γ�Ⱦ��ȸ߶ȣ���д�뵽cell��
    gstandaccessinfo{i+1,1} = gst.InstanceName;
    gstandaccessinfo{i+1,2} = gst.Position.QueryPlanetodeticArray;
    %����ɼ��ԣ���ȡ�����ǿɼ�������ÿ���ɼ����ε���ֹʱ�䡢����ʱ��
    %��д�뵽cell�У��Ա�������
    gst2sataccess = gst.GetAccessToObject(pursue);
    gst2sataccess.ComputeAccess();
    gstandaccessinfo{i+1,3} = gst2sataccess.ComputedAccessIntervalTimes.Count;
    %��ȡ
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


%% -------------------- % ����
% �������� pָ��t
pursue2targaccess = pursue.GetAccessToObject(targ);
pursue2targaccess.ComputeAccess()
% P2T_AER = pursue2targaccess.DataProviders.Item('Points(ICRF)').Group.Item('ToObject').Exec(scn.StartTime,scn.StopTime,120);
% P2T_RightA = cell2mat(P2T_AER.DataSets.GetDataSetByName('RightAscension').GetValues);
% P2T_Decl = cell2mat(P2T_AER.DataSets.GetDataSetByName('Declination').GetValues);
% P2T_ran = cell2mat(P2T_AER.DataSets.GetDataSetByName('Magnitude').GetValues);
% P2T_time = cell2mat(P2T_AER.DataSets.GetDataSetByName('Time').GetValues);
% ������;�б������赲�������AER����ʹ��
% P2T_AER = pursue2targaccess.DataProviders.Item('AER Data').Group.Item('VVLH CBF').Exec(str,sto,120);
% P2T_azi = cell2mat(P2T_AER.DataSets.GetDataSetByName('Azimuth').GetValues);
% P2T_ele = cell2mat(P2T_AER.DataSets.GetDataSetByName('Elevation').GetValues);
% P2T_ran = cell2mat(P2T_AER.DataSets.GetDataSetByName('Range').GetValues);
% P2T_time = cell2mat(P2T_AER.DataSets.GetDataSetByName('Time').GetValues);



%% -------------------- % ����Լ��
% ̫������ pָ��sun
name = 'Sun' ;
sun = scn.Children.Item(name);
pursue2sun_access = pursue.GetAccessToObject(sun);
pursue2sun_access.ComputeAccess()
% ��ȡ���ʵļ����
pur_sun_accessinfo = cell(1,2);
pur_sun_accessinfo{1,1} = 'AER';
pur_sun_accessinfo{1,2} = '�ɼ�������ֹ��';
% ����AER
access_aer = cell(pursue2sun_access.ComputedAccessIntervalTimes.Count,1);
% ������ֹ
access_st_end = cell(pursue2sun_access.ComputedAccessIntervalTimes.Count,1);
for j = 1:pursue2sun_access.ComputedAccessIntervalTimes.Count
    [str,sto] = pursue2sun_access.ComputedAccessIntervalTimes.GetInterval(j-1);
    pursue2sun_access. AccessTimePeriod = 'eUserSpecAccessTime';
    pursue2sun_access.SpecifyAccessTimePeriod(str,sto)
    %     P2S_AER = pursue2sun_access.DataProviders.Item('AER Data').Group.Item('VVLH CBF').Exec(str,sto,120);
    %     P2S_azi = cell2mat(P2S_AER.DataSets.GetDataSetByName('Azimuth').GetValues);
    %     P2S_ele = cell2mat(P2S_AER.DataSets.GetDataSetByName('Elevation').GetValues);
    %     P2S_ran = cell2mat(P2S_AER.DataSets.GetDataSetByName('Range').GetValues);
    %     P2S_time = cell2mat(P2S_AER.DataSets.GetDataSetByName('Time').GetValues);
    % ��������Ѿ���RA��DC�ˣ�����AER��ֻ��Ϊ��ͳһ����
    P2S_AER = pursue2sun_access.DataProviders.Item('Points(ICRF)').Group.Item('ToObject').Exec(str,sto,120);
    P2S_azi = cell2mat(P2S_AER.DataSets.GetDataSetByName('RightAscension').GetValues);
    P2S_ele = cell2mat(P2S_AER.DataSets.GetDataSetByName('Declination').GetValues);
    P2S_time = cell2mat(P2S_AER.DataSets.GetDataSetByName('Time').GetValues);
    access_aer{j}= {P2S_time,P2S_azi,P2S_ele};
    accessduration = pursue2sun_access.AccessTimePeriodData.Duration;
    access_st_end{j}= {str,sto,accessduration};
end
% ̫����
pur_sun_accessinfo{2,1} = access_aer;
% ̫��ʱ��
pur_sun_accessinfo{2,2} = access_st_end;
%% ������������������������������������ ����ǰ��̫����
pursue2sun_access.RemoveAccess();
sun_angle_end = [access_aer{end}{2}(end),access_aer{end}{3}(end)];
sun_vec =  an2vec(sun_angle_end(1),sun_angle_end(2));
sat_vec = vtf - vpf;
fprintf('�ӽ�̫����/deg')
sun_vec_angle = acos(dot(sat_vec,sun_vec)/(norm(sat_vec)*norm(sun_vec)))*180/pi
fprintf('�ն˾���/km')
dist_final = norm(rpf-rtf)
%% ---------------------------------- ���� ----------------------------------
sun_period = zeros(1,2); % ������1��
sun_c_i = 1;
for sun_i = 2:2  % ��һ������������,��������վ
    for time_i = 1:size(pur_sun_accessinfo{sun_i,2},1) % ����ĳ��վ���л���
        timesuneriod = pur_sun_accessinfo{sun_i, 2}{time_i, 1};
        dateTime1 = datevec(datetime(timesuneriod{1}, 'InputFormat', 'dd MMM yyyy HH:mm:ss.SSS' ,'Locale', 'en_US')); % ��char ��date ��s
        dateTime2 = datevec(datetime(timesuneriod{2}, 'InputFormat', 'dd MMM yyyy HH:mm:ss.SSS' ,'Locale', 'en_US'));
        sun_period(sun_c_i,:) = [etime(dateTime1, t0_sec),etime(dateTime2, t0_sec)];
        sun_c_i = sun_c_i+1;
    end
end
% ���պϼ�
fprintf('������ʱ��Σ�');
sun_union = interval_union(sun_period)

%% ---------------------------------- ��� ----------------------------------

% ���Լ��
% ��ȡ���
sp_period = zeros(size(spst_oe,1),2); % ������size(spst_oe,1)��
sp_c_i = 1;
for sp_i = 2: size(spst_oe,1)+1  % ��һ������������,��������վ
    for time_i = 1:size(spstandaccessinfo{sp_i,4},1) % ����ĳ��վ���л���
        timesperiod = spstandaccessinfo{sp_i, 4}{time_i, 1};
        dateTime1 = datevec(datetime(timesperiod{1}, 'InputFormat', 'dd MMM yyyy HH:mm:ss.SSS' ,'Locale', 'en_US')); % ��char ��date ��s
        dateTime2 = datevec(datetime(timesperiod{2}, 'InputFormat', 'dd MMM yyyy HH:mm:ss.SSS' ,'Locale', 'en_US'));
        sp_period(sp_c_i,:) = [etime(dateTime1, t0_sec),etime(dateTime2, t0_sec)];
        sp_c_i = sp_c_i+1;
    end
end

% ��ȡ�ػ�
gr_period = zeros(1,2); % ������1��
gr_c_i = 1;
for sp_i = 2:2  % ���ж�ȡ ���г�����ֻ��һ������վ,�����Ϊ��size(loc,1)
    for time_i = 1:size(gstandaccessinfo{sp_i,4},1) % ����ĳ��վ���л���
        timesperiod = gstandaccessinfo{sp_i, 4}{time_i, 1};
        dateTime1 = datevec(datetime(timesperiod{1}, 'InputFormat', 'dd MMM yyyy HH:mm:ss.SSS' ,'Locale', 'en_US')); % ��char ��date ��s
        dateTime2 = datevec(datetime(timesperiod{2}, 'InputFormat', 'dd MMM yyyy HH:mm:ss.SSS' ,'Locale', 'en_US'));
        gr_period(gr_c_i,:) = [etime(dateTime1, t0_sec),etime(dateTime2, t0_sec)];
        gr_c_i = gr_c_i+1;
    end
end

% ��غϼ�
total_period = [sp_period;gr_period];
fprintf('�����ʱ��Σ�');
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

