UNIT simplerPhysics;

{$mode objfpc}{$H+}
INTERFACE
USES basicGraphics, serializationUtil;
CONST
  SPATIAL_SCALE=0.25;

  FOOD_CAP=10;
  FIRE_CAP=10;
  INV_FOOD_CAP=1/FOOD_CAP;
  INV_FIRE_CAP=1/FIRE_CAP;

  TIME_STEP_SIZE =0.5;

TYPE TmyFloat=double;
TYPE
  T_cell=record
    fire,food:TmyFloat;
  end;
  T_equationParameters=record
    CONSUMER_MOVEMENT_FACTOR  :double;
    FOOD_DIFFUSION_COEFFICIENT:double;
    GROWTH_THRESHOLD          :double;
    DIE_THRESHOLD             :double;
    GROWTH_SPEED              :double;
    DIE_SPEED                 :double;
    CONSUMPTION_SPEED         :double;
    GROW_BACK_FACTOR          :double;
  end;

  T_State=array[0..SYS_HEIGHT-1,0..SYS_WIDTH-1] of T_cell;
  T_timeIntegrationMethod=(HEUN_EULER, BOGACKI_SHAMPINE, CASH_CARP, BSB_SSP_3, BSB_SSP_4, MULTISTEP);

  T_spatial_discretization=(FIRST_ORDER_UPWIND,SECOND_ORDER_CENTRAL,THIRD_ORDER_UPWIND,LIMITED_THIRD_ORDER_UPWIND);
  T_derivativeFunction=FUNCTION(CONST s:T_State):T_State;
  {$ifdef multithreading}
  T_snapshotState=(ss_none,ss_snapshot_queried,ss_set_posted);
  {$endif}

  { T_userInteraction }
  T_statistics=record
    minFood,
    maxFood,
    avgFood,
    minFire,
    maxFire,
    avgFire:TmyFloat;
  end;

  T_userInteraction=object
    private
      {$ifdef multithreading}
      snapshot:T_State;
      snapshotState:T_snapshotState;
      {$endif}
      lastTimer:qword;
      igniteRate:TmyFloat;
      igniting:shortint;
      pointCount:longint;
      points:array[0..63] of record
        ix,iy:longint;
        ir:TmyFloat;
      end;
      equation_parameters:T_equationParameters;
      currentStats:T_statistics;
      settingChanged:boolean;
      spatialDiscretization:T_spatial_discretization;
      timeIntegrationMethod:T_timeIntegrationMethod;
      multistep_order,multistep_steps:byte;
      {$ifdef multithreading}
      usercs:TRTLCriticalSection;
      {$endif}
      errorTolerance:TmyFloat;
    public
      recording:boolean;
      CONSTRUCTOR create;
      DESTRUCTOR destroy;
      PROCEDURE startIgniting(CONST x,y:longint);
      PROCEDURE startExtinguishing(CONST x,y:longint);
      PROCEDURE addPoint(CONST x,y:longint);
      PROCEDURE affirmLastPoint;
      PROCEDURE logStatistics(CONST stats:T_statistics);
      FUNCTION  getStatistics:T_statistics;
      PROCEDURE stop;
      FUNCTION isStopped:boolean;
      PROCEDURE logSettingChanged;
      PROCEDURE setTimeIntegrationMethod(CONST mehod:T_timeIntegrationMethod; CONST ms_steps,ms_order:longint);
      PROCEDURE setSpatialDiscretization(CONST spatial_discretization:T_spatial_discretization);
      PROCEDURE setErrorTolerance(CONST newTol:TmyFloat);
      {$ifdef multithreading}
      FUNCTION getSnapshot:T_State;
      PROCEDURE setState(CONST state:T_State);
      FUNCTION snapshotQueried:boolean;
      {$endif}

      PROCEDURE set_CONSUMER_MOVEMENT_FACTOR  (CONST val:TmyFloat);
      PROCEDURE set_FOOD_DIFFUSION_COEFFICIENT(CONST val:TmyFloat);
      PROCEDURE set_GROWTH_THRESHOLD          (CONST val:TmyFloat);
      PROCEDURE set_DIE_THRESHOLD             (CONST val:TmyFloat);
      PROCEDURE set_GROWTH_SPEED              (CONST val:TmyFloat);
      PROCEDURE set_DIE_SPEED                 (CONST val:TmyFloat);
      PROCEDURE set_CONSUMPTION_SPEED         (CONST val:TmyFloat);
      PROCEDURE set_GROW_BACK_FACTOR          (CONST val:TmyFloat);
  end;

  { T_cellSystem }
  P_cellSystem=^T_cellSystem;
  T_cellSystem=object
    private
      timeStepIndex:longint;
      recommendedStepSize:TmyFloat;
      prevDt:array[0..5] of TmyFloat;
      D:T_derivativeFunction;

      FUNCTION getFood(CONST x,y:longint):TmyFloat;
      PROCEDURE setFood(CONST x,y:longint; CONST value:TmyFloat);
      FUNCTION getFire(CONST x,y:longint):TmyFloat;
      PROCEDURE setFire(CONST x,y:longint; CONST value:TmyFloat);
      PROCEDURE modifyFire(CONST x,y:longint; CONST absoluteDelta:TmyFloat);
    public
      state:T_State;
      CONSTRUCTOR create;
      DESTRUCTOR destroy;

      PROCEDURE doMacroTimeStep(VAR user:T_userInteraction);
      PROCEDURE getPicture(VAR rgbPicture:T_rgbPicture);

      PROPERTY food[x,y:longint]:TmyFloat read getFood write setFood;
      PROPERTY fire[x,y:longint]:TmyFloat read getFire write setFire;
      PROPERTY stepIndex:longint read timeStepIndex;
  end;

  P_snapshot=^T_snapshot;

  { T_snapshot }

  T_snapshot=object(T_serializable)
    name:string;
    state:T_State;
    physics:T_equationParameters;
    spatialDiscretization:T_spatial_discretization;
    timeIntegrationMethod:T_timeIntegrationMethod;
    msOrder,msSteps:byte;
    toleranceFactor:double;

    created_successfully:boolean;

    CONSTRUCTOR create(VAR user:T_userInteraction{$ifndef multithreading}; VAR sys:T_cellSystem{$endif});
    CONSTRUCTOR createToLoad;
    DESTRUCTOR destroy;

    PROCEDURE apply(VAR user:T_userInteraction; {$ifndef multithreading}VAR sys:T_cellSystem;{$endif} CONST includeState,includePhysicsParameters,includeDiscretization:boolean);
    FUNCTION loadFromStream(VAR stream:T_bufferedInputStreamWrapper):boolean; virtual;
    PROCEDURE saveToStream(VAR stream:T_bufferedOutputStreamWrapper); virtual;
  end;

  { T_persistedState }

  T_persistedState=object(T_serializable)
    snapshots:array of P_snapshot;
    {$ifdef enable_calibration}
    calibration_handle:textFile;
    calibration_open:boolean;
    {$endif}

    CONSTRUCTOR create;
    DESTRUCTOR destroy;

    PROCEDURE addSnapshot(VAR user:T_userInteraction{$ifndef multithreading}; VAR sys:T_cellSystem{$endif});
    PROCEDURE remSnapshot(CONST snapshotIndex:longint);

    FUNCTION getSerialVersion:dword; virtual;
    FUNCTION loadFromStream(VAR stream:T_bufferedInputStreamWrapper):boolean; virtual;
    PROCEDURE saveToStream(VAR stream:T_bufferedOutputStreamWrapper); virtual;

    {$ifdef enable_calibration}
    PROCEDURE addCalibrationData(CONST method:T_timeIntegrationMethod; CONST steps,order:longint; CONST initialState, approximation:T_State; CONST D:T_derivativeFunction; CONST dt,estimatedError:TmyFloat);
    {$endif}
  end;

VAR persistedState:T_persistedState;
FUNCTION get_equationParameters:T_equationParameters;
IMPLEMENTATION
USES sysutils,math;
VAR equationParameters:T_equationParameters=(
    CONSUMER_MOVEMENT_FACTOR  : SPATIAL_SCALE;
    FOOD_DIFFUSION_COEFFICIENT:-SPATIAL_SCALE;
    GROWTH_THRESHOLD          :1;
    DIE_THRESHOLD             :0.25;
    GROWTH_SPEED              :0.5;
    DIE_SPEED                 :10;
    CONSUMPTION_SPEED         :0.01;
    GROW_BACK_FACTOR          :0.01);

FUNCTION get_equationParameters:T_equationParameters; begin result:=equationParameters  ; end;
{ T_persistedState }

CONSTRUCTOR T_persistedState.create;
  begin
    if not(loadFromFile(ChangeFileExt(paramStr(0),'.cfg'))) then begin
      setLength(snapshots,0);
    end;
    {$ifdef enable_calibration} calibration_open:=false; {$endif}
  end;

DESTRUCTOR T_persistedState.destroy;
  begin
    {$ifdef enable_calibration} if calibration_open then close(calibration_handle); {$endif}
    saveToFile(ChangeFileExt(paramStr(0),'.cfg'));
  end;

PROCEDURE T_persistedState.addSnapshot(VAR user: T_userInteraction{$ifndef multithreading}; VAR sys:T_cellSystem{$endif});
  VAR newSnapshot:P_snapshot;
  begin
    new(newSnapshot,create(user{$ifndef multithreading},sys{$endif}));
    if newSnapshot^.created_successfully then begin
      setLength(snapshots,length(snapshots)+1);
      snapshots[length(snapshots)-1]:=newSnapshot;
      newSnapshot^.name:='<snapshot #'+intToStr(length(snapshots))+'>';
    end else dispose(newSnapshot,destroy);
  end;

PROCEDURE T_persistedState.remSnapshot(CONST snapshotIndex: longint);
  VAR i:longint;
  begin
    if (snapshotIndex<0) or (snapshotIndex>=length(snapshots)) then exit;
    dispose(snapshots[snapshotIndex],destroy);
    for i:=snapshotIndex to length(snapshots)-2 do snapshots[i]:=snapshots[i+1];
    setLength(snapshots,length(snapshots)-1);
  end;

FUNCTION T_persistedState.getSerialVersion: dword;
  begin
    result:=0;
  end;

FUNCTION T_persistedState.loadFromStream(VAR stream: T_bufferedInputStreamWrapper): boolean;
  VAR snapshotCount,i:longint;
  begin
    if not(inherited) then exit(false);
    snapshotCount:=stream.readNaturalNumber;
    result:=stream.allOkay;
    i:=0;
    while result and (i<snapshotCount) do begin
      setLength(snapshots,i+1);
      new(snapshots[i],createToLoad);
      result:=result and snapshots[i]^.loadFromStream(stream) and stream.allOkay;
      inc(i);
    end;
    if not(result) then begin
      for i:=0 to length(snapshots)-1 do dispose(snapshots[i],destroy);
      setLength(snapshots,0);
    end;
  end;

PROCEDURE T_persistedState.saveToStream(VAR stream: T_bufferedOutputStreamWrapper);
  VAR i:longint;
  begin
    inherited;
    stream.writeNaturalNumber(length(snapshots));
    for i:=0 to length(snapshots)-1 do snapshots[i]^.saveToStream(stream);
  end;

{ T_snapshot }

CONSTRUCTOR T_snapshot.create(VAR user: T_userInteraction{$ifndef multithreading}; VAR sys:T_cellSystem{$endif});
  {$ifdef multithreading}
  CONST MAX_WAIT=500;
  VAR query_counter:longint=0;
  {$endif}
  begin
    {$ifdef multithreading}
    with user do begin
      enterCriticalSection(usercs);
      snapshotState:=ss_snapshot_queried;
      leaveCriticalSection(usercs);
    end;
    {$else}
    state:=sys.state;
    {$endif}
    physics:=equationParameters;
    spatialDiscretization:=user.spatialDiscretization;
    timeIntegrationMethod:=user.timeIntegrationMethod;
    toleranceFactor:=user.errorTolerance;
    msSteps:=user.multistep_steps;
    msOrder:=user.multistep_order;
    {$ifdef multithreading}
    created_successfully:=false;
    with user do begin
      enterCriticalSection(usercs);
      while (snapshotState=ss_snapshot_queried) and (query_counter<MAX_WAIT) do begin
        leaveCriticalSection(usercs);
        inc(query_counter);
        sleep(1);
        enterCriticalSection(usercs);
      end;
      state:=snapshot;
      leaveCriticalSection(usercs);
    end;
    created_successfully:=query_counter<MAX_WAIT;
    {$else}
    created_successfully:=true;
    {$endif}
  end;

CONSTRUCTOR T_snapshot.createToLoad;
  begin
  end;

DESTRUCTOR T_snapshot.destroy;
  begin
  end;

PROCEDURE T_snapshot.apply(VAR user: T_userInteraction; {$ifndef multithreading}VAR sys:T_cellSystem;{$endif} CONST includeState,includePhysicsParameters, includeDiscretization: boolean);
  begin
    {$ifdef multithreading}
    enterCriticalSection(user.usercs);
    {$endif}
    if includeState
    then {$ifdef multithreading} user.setState(state); {$else} sys.state:=state; {$endif}
    if includePhysicsParameters
    then user.equation_parameters:=physics;
    if includeDiscretization
    then begin
      user.spatialDiscretization:=spatialDiscretization;
      user.timeIntegrationMethod:=timeIntegrationMethod;
      user.errorTolerance       :=toleranceFactor;
    end;
    {$ifdef multithreading}
    user.settingChanged:=true;
    leaveCriticalSection(user.usercs);
    {$endif}
  end;

FUNCTION T_snapshot.loadFromStream(VAR stream: T_bufferedInputStreamWrapper): boolean;
  begin
    name:=stream.readAnsiString;
    stream.read(state,sizeOf(state));
    stream.read(physics,sizeOf(physics));
    stream.read(spatialDiscretization,sizeOf(spatialDiscretization));
    stream.read(timeIntegrationMethod,sizeOf(timeIntegrationMethod));
    msSteps:=stream.readByte([2..6]);
    msOrder:=stream.readByte([2..6]);
    toleranceFactor:=stream.readDouble;
    result:=stream.allOkay;
  end;

PROCEDURE T_snapshot.saveToStream(VAR stream: T_bufferedOutputStreamWrapper);
  begin
    stream.writeAnsiString(name);
    stream.write(state,sizeOf(state));
    stream.write(physics,sizeOf(physics));
    stream.write(spatialDiscretization,sizeOf(spatialDiscretization));
    stream.write(timeIntegrationMethod,sizeOf(timeIntegrationMethod));
    stream.writeByte(msSteps);
    stream.writeByte(msOrder);
    stream.writeDouble(toleranceFactor);
  end;

{ T_userInteraction }

CONSTRUCTOR T_userInteraction.create;
  begin
    {$ifdef multithreading}
    initCriticalSection(usercs);
    snapshotState:=ss_none;
    {$endif}
    igniting:=0;
    settingChanged:=true;
    timeIntegrationMethod:=BOGACKI_SHAMPINE;
    multistep_order:=3;
    multistep_steps:=3;
    errorTolerance:=1;
    recording:=false;
    spatialDiscretization:=LIMITED_THIRD_ORDER_UPWIND;
    equation_parameters  :=equationParameters;
  end;

DESTRUCTOR T_userInteraction.destroy;
  begin
    {$ifdef multithreading}
    doneCriticalSection(usercs);
    {$endif}
  end;

PROCEDURE T_userInteraction.startIgniting(CONST x, y: longint);
  begin
    {$ifdef multithreading}
    enterCriticalSection(usercs);
    {$endif}
    igniting:=1;
    igniteRate:=0.1;
    lastTimer:=GetTickCount64;
    pointCount:=1;
    points[0].ix:=x;
    points[0].iy:=y;
    points[0].ir:=igniteRate;
    {$ifdef multithreading}
    leaveCriticalSection(usercs);
    {$endif}
  end;

PROCEDURE T_userInteraction.startExtinguishing(CONST x, y: longint);
  begin
    {$ifdef multithreading}
    enterCriticalSection(usercs);
    {$endif}
    igniting:=-1;
    igniteRate:=-0.1;
    lastTimer:=GetTickCount64;
    pointCount:=1;
    points[0].ix:=x;
    points[0].iy:=y;
    points[0].ir:=igniteRate;
    {$ifdef multithreading}
    leaveCriticalSection(usercs);
    {$endif}
  end;

PROCEDURE T_userInteraction.addPoint(CONST x, y: longint);
  begin
    if (igniting=0) or (pointCount>=length(points)) then exit;
    {$ifdef multithreading}
    enterCriticalSection(usercs);
    {$endif}
    igniteRate+=(GetTickCount64-lastTimer)*5E-3*igniting;
    lastTimer:=GetTickCount64;
    if (pointCount>0) and (points[pointCount-1].ix=x) and (points[pointCount-1].iy=y)
    then points[pointCount-1].ir+=igniteRate
    else begin
      points[pointCount].ix:=x;
      points[pointCount].iy:=y;
      points[pointCount].ir:=igniteRate;
      inc(pointCount);
    end;
    {$ifdef multithreading}
    leaveCriticalSection(usercs);
    {$endif}
  end;

PROCEDURE T_userInteraction.affirmLastPoint;
  begin
    if (igniting=0) or (pointCount=0) then exit;
    igniteRate+=(GetTickCount64-lastTimer)*5E-3*igniting;
    lastTimer:=GetTickCount64;
    points[pointCount-1].ir+=igniteRate;
  end;

PROCEDURE T_userInteraction.logStatistics(CONST stats: T_statistics);
  begin
    currentStats:=stats;
  end;

FUNCTION T_userInteraction.getStatistics: T_statistics;
  begin
    result:=currentStats;
  end;

PROCEDURE T_userInteraction.stop;
  begin
    igniting:=0;
  end;

FUNCTION T_userInteraction.isStopped: boolean;
  begin
    result:=igniting=0;
  end;

PROCEDURE T_userInteraction.logSettingChanged;
  begin
    {$ifdef multithreading}
    enterCriticalSection(usercs);
    {$endif}
    settingChanged:=true;
    {$ifdef multithreading}
    leaveCriticalSection(usercs);
    {$endif}
  end;

PROCEDURE T_userInteraction.setTimeIntegrationMethod(CONST mehod: T_timeIntegrationMethod; CONST ms_steps,ms_order:longint);
  begin
    {$ifdef multithreading}
    enterCriticalSection(usercs);
    {$endif}
    settingChanged:=true;
    timeIntegrationMethod:=mehod;
    multistep_steps:=ms_steps;
    multistep_order:=ms_order;
    {$ifdef multithreading}
    leaveCriticalSection(usercs);
    {$endif}
  end;

PROCEDURE T_userInteraction.setSpatialDiscretization(CONST spatial_discretization: T_spatial_discretization);
  begin
    {$ifdef multithreading}
    enterCriticalSection(usercs);
    {$endif}
    spatialDiscretization:=spatial_discretization;
    settingChanged:=true;
    {$ifdef multithreading}
    leaveCriticalSection(usercs);
    {$endif}
  end;

PROCEDURE T_userInteraction.setErrorTolerance(CONST newTol:TmyFloat);
  begin
    {$ifdef multithreading}
    enterCriticalSection(usercs);
    {$endif}
    errorTolerance:=newTol;
    settingChanged:=true;
    {$ifdef multithreading}
    leaveCriticalSection(usercs);
    {$endif}
  end;

PROCEDURE T_userInteraction.set_CONSUMER_MOVEMENT_FACTOR(CONST val: TmyFloat);
  begin
    {$ifdef multithreading}enterCriticalSection(usercs);{$endif}
    equation_parameters.CONSUMER_MOVEMENT_FACTOR:=val;
    settingChanged:=true;
    {$ifdef multithreading}leaveCriticalSection(usercs);{$endif}
  end;

PROCEDURE T_userInteraction.set_FOOD_DIFFUSION_COEFFICIENT(CONST val: TmyFloat);
  begin
    {$ifdef multithreading}enterCriticalSection(usercs);{$endif}
    equation_parameters.FOOD_DIFFUSION_COEFFICIENT:=val;
    settingChanged:=true;
    {$ifdef multithreading}leaveCriticalSection(usercs);{$endif}
  end;

PROCEDURE T_userInteraction.set_GROWTH_THRESHOLD(CONST val: TmyFloat);
  begin
    {$ifdef multithreading}enterCriticalSection(usercs);{$endif}
    equation_parameters.GROWTH_THRESHOLD:=val;
    settingChanged:=true;
    {$ifdef multithreading}leaveCriticalSection(usercs);{$endif}
  end;

PROCEDURE T_userInteraction.set_DIE_THRESHOLD(CONST val: TmyFloat);
  begin
    {$ifdef multithreading}enterCriticalSection(usercs);{$endif}
    equation_parameters.DIE_THRESHOLD:=val;
    settingChanged:=true;
    {$ifdef multithreading}leaveCriticalSection(usercs);{$endif}
  end;

PROCEDURE T_userInteraction.set_GROWTH_SPEED(CONST val: TmyFloat);
  begin
    {$ifdef multithreading}enterCriticalSection(usercs);{$endif}
    equation_parameters.GROWTH_SPEED:=val;
    settingChanged:=true;
    {$ifdef multithreading}leaveCriticalSection(usercs);{$endif}
  end;

PROCEDURE T_userInteraction.set_DIE_SPEED(CONST val: TmyFloat);
  begin
    {$ifdef multithreading}enterCriticalSection(usercs);{$endif}
    equation_parameters.DIE_SPEED:=val;
    settingChanged:=true;
    {$ifdef multithreading}leaveCriticalSection(usercs);{$endif}
  end;

PROCEDURE T_userInteraction.set_CONSUMPTION_SPEED(CONST val: TmyFloat);
  begin
    {$ifdef multithreading}enterCriticalSection(usercs);{$endif}
    equation_parameters.CONSUMPTION_SPEED:=val;
    settingChanged:=true;
    {$ifdef multithreading}leaveCriticalSection(usercs);{$endif}
  end;

PROCEDURE T_userInteraction.set_GROW_BACK_FACTOR(CONST val: TmyFloat);
  begin
    {$ifdef multithreading}enterCriticalSection(usercs);{$endif}
    equation_parameters.GROW_BACK_FACTOR:=val;
    settingChanged:=true;
    {$ifdef multithreading}leaveCriticalSection(usercs);{$endif}
  end;

{$ifdef multithreading}
FUNCTION T_userInteraction.getSnapshot:T_State;
  begin
    enterCriticalSection(usercs);
    snapshotState:=ss_snapshot_queried;
    while snapshotState=ss_snapshot_queried do begin
      leaveCriticalSection(usercs);
      sleep(1);
      enterCriticalSection(usercs);
    end;
    result:=snapshot;
    leaveCriticalSection(usercs);
  end;

PROCEDURE T_userInteraction.setState(CONST state:T_State);
  begin
    enterCriticalSection(usercs);
    snapshot:=state;
    snapshotState:=ss_set_posted;
    leaveCriticalSection(usercs);
  end;

FUNCTION T_userInteraction.snapshotQueried:boolean;
  begin
    enterCriticalSection(usercs);
    result:=(snapshotState=ss_snapshot_queried);
    leaveCriticalSection(usercs);
  end;
{$endif}

{ T_cellSystem }

FUNCTION T_cellSystem.getFood(CONST x, y: longint): TmyFloat;
  begin
    result:=state[y,x].food;
  end;

PROCEDURE T_cellSystem.setFood(CONST x, y: longint; CONST value: TmyFloat);
  begin
    state[y,x].food:=value;
  end;

FUNCTION T_cellSystem.getFire(CONST x, y: longint): TmyFloat;
  begin
    result:=state[y,x].fire;
  end;

PROCEDURE T_cellSystem.setFire(CONST x, y: longint; CONST value: TmyFloat);
  begin
    state[y,x].fire:=value;
  end;

PROCEDURE T_cellSystem.modifyFire(CONST x, y: longint; CONST absoluteDelta: TmyFloat);
  VAR f:TmyFloat;
  begin
    if (x<0) or (x>=SYS_WIDTH) or (y<0) or (y>=SYS_HEIGHT) then exit;
    f:=state[y,x].fire+absoluteDelta;
    if f<0 then f:=0 else if f>FIRE_CAP then f:=FIRE_CAP;
    state[y,x].fire:=f;
  end;

DESTRUCTOR T_cellSystem.destroy;
  begin
  end;

FUNCTION cellChemistry(CONST s:T_cell):T_cell; {$ifndef debugMode} inline; {$endif}
  begin
    result.food:=abs(s.food)*(equationParameters.GROW_BACK_FACTOR*(1-s.food*INV_FOOD_CAP)-s.fire*equationParameters.CONSUMPTION_SPEED);
    if s.food<equationParameters.DIE_THRESHOLD
    then result.fire:=(s.food-equationParameters.DIE_THRESHOLD) *equationParameters.DIE_SPEED   *s.fire
    else if s.food>equationParameters.GROWTH_THRESHOLD
    then result.fire:= (s.food-equationParameters.GROWTH_THRESHOLD) *equationParameters.GROWTH_SPEED*abs(s.fire)*(1-s.fire*INV_FIRE_CAP)
    else result.fire:=0;
  end;

FUNCTION D_1stOrderUpwind(CONST s:T_State):T_State;
  PROCEDURE calculateFlow(CONST food0,food1:TmyFloat; OUT foodFlow:TmyFloat;
                          CONST fire0,fire1:TmyFloat; OUT fireFlow:TmyFloat); inline;
    begin
      fireFlow:=equationParameters.CONSUMER_MOVEMENT_FACTOR  *(food1-food0); if fireFlow>0 then fireFlow*=fire0 else fireFlow*=fire1;
      foodFlow:=equationParameters.FOOD_DIFFUSION_COEFFICIENT*(fire1-fire0); if foodFlow>0 then foodFlow*=food0 else foodFlow*=food1;
    end;
  VAR i,j:longint;
      dFood, dFire: TmyFloat;
  begin
    for i:=0 to SYS_WIDTH-1 do result[0,i]:=cellChemistry(s[0,i]);
    for j:=0 to SYS_HEIGHT-1 do begin
      for i:=0 to SYS_WIDTH-1 do begin
        if i<SYS_WIDTH-1 then begin
          calculateFlow(s[j,i].food,s[j,i+1].food,dFood,
                        s[j,i].fire,s[j,i+1].fire,dFire);
          result[j,i  ].food-=dFood;
          result[j,i  ].fire-=dFire;
          result[j,i+1].food+=dFood;
          result[j,i+1].fire+=dFire;
        end;
        if j<SYS_HEIGHT-1 then begin
          result[j+1,i]:=cellChemistry(s[j+1,i]);
          calculateFlow(s[j,i].food,s[j+1,i].food,dFood,
                        s[j,i].fire,s[j+1,i].fire,dFire);
          result[j  ,i].food-=dFood;
          result[j  ,i].fire-=dFire;
          result[j+1,i].food+=dFood;
          result[j+1,i].fire+=dFire;
        end;
      end;
    end;
  end;

FUNCTION D_2ndOrderCentral(CONST s:T_State):T_State;
  PROCEDURE calculateFlow(CONST food0,food1:TmyFloat; OUT foodFlow:TmyFloat;
                          CONST fire0,fire1:TmyFloat; OUT fireFlow:TmyFloat); inline;
    begin
      fireFlow:=equationParameters.CONSUMER_MOVEMENT_FACTOR  *(food1-food0)*0.5*(fire0+fire1);
      foodFlow:=equationParameters.FOOD_DIFFUSION_COEFFICIENT*(fire1-fire0)*0.5*(food0+food1);
    end;
  VAR i,j:longint;
      dFood, dFire: TmyFloat;
  begin
    for i:=0 to SYS_WIDTH-1 do result[0,i]:=cellChemistry(s[0,i]);
    for j:=0 to SYS_HEIGHT-1 do begin
      for i:=0 to SYS_WIDTH-1 do begin
        if i<SYS_WIDTH-1 then begin
          calculateFlow(s[j,i].food,s[j,i+1].food,dFood,
                        s[j,i].fire,s[j,i+1].fire,dFire);
          result[j,i  ].food-=dFood;
          result[j,i  ].fire-=dFire;
          result[j,i+1].food+=dFood;
          result[j,i+1].fire+=dFire;
        end;
        if j<SYS_HEIGHT-1 then begin
          result[j+1,i]:=cellChemistry(s[j+1,i]);
          calculateFlow(s[j,i].food,s[j+1,i].food,dFood,
                        s[j,i].fire,s[j+1,i].fire,dFire);
          result[j  ,i].food-=dFood;
          result[j  ,i].fire-=dFire;
          result[j+1,i].food+=dFood;
          result[j+1,i].fire+=dFire;
        end;
      end;
    end;
  end;

FUNCTION D_3rdOrderUpwind(CONST s:T_State):T_State;
  VAR i,j:longint;
      ip,im,jp,jm:longint;
      dFood,dFire: TmyFloat;

  PROCEDURE calculateFlow(CONST food_1,food0,food1,food2:TmyFloat; OUT foodFlow:TmyFloat;
                          CONST fire_1,fire0,fire1,fire2:TmyFloat; OUT fireFlow:TmyFloat);  {$ifndef debugMode} inline; {$endif}
    VAR slopeRatio:TmyFloat;
    {$MACRO ON}
    {$define x0:=fire_1}
    {$define x1:=fire0}
    {$define x2:=fire1}
    {$define fluxLimitedUpwind:=begin
        slopeRatio:=-1/6*x0+5/6*x1+1/3*x2;
      end}
    begin
      fireFlow:=equationParameters.CONSUMER_MOVEMENT_FACTOR  *((food1-food0)*5/4+(food2-food_1)*-1/12);
      if fireFlow>0
      then begin
        {$define x0:=fire_1}
        {$define x1:=fire0}
        {$define x2:=fire1}
        fluxLimitedUpwind;
      end else begin
        {$define x0:=fire2}
        {$define x1:=fire1}
        {$define x2:=fire0}
        fluxLimitedUpwind;
      end;
      fireFlow*=slopeRatio;
      foodFlow:=equationParameters.FOOD_DIFFUSION_COEFFICIENT*((fire1-fire0)*5/4+(fire2-fire_1)*-1/12);
      if foodFlow>0
      then begin
        {$define x0:=food_1}
        {$define x1:=food0}
        {$define x2:=food1}
        fluxLimitedUpwind;
      end else begin
        {$define x0:=food2}
        {$define x1:=food1}
        {$define x2:=food0}
        fluxLimitedUpwind;
      end;
      foodFlow*=slopeRatio;
    end;

  begin
    for j:=0 to 1 do
    for i:=0 to SYS_WIDTH-1 do result[j,i]:=cellChemistry(s[j,i]);
    for j:=0 to SYS_HEIGHT-1 do begin

      jm:=j-1; if jm< 0          then jm:=0;
      jp:=j+2; if jp>=SYS_HEIGHT
      then jp:=SYS_HEIGHT-1
      else for i:=0 to SYS_WIDTH-1 do result[jp,i]:=cellChemistry(s[jp,i]);

      for i:=0 to SYS_WIDTH-1 do begin
        im:=i-1; if im< 0         then im:=0;
        ip:=i+2; if ip>=SYS_WIDTH then ip:=SYS_WIDTH-1;
        if i<SYS_WIDTH-1 then begin
          calculateFlow(s[j,im].food,s[j,i].food,s[j,i+1].food,s[j,ip].food,dFood,
                        s[j,im].fire,s[j,i].fire,s[j,i+1].fire,s[j,ip].fire,dFire);
          result[j,i  ].food-=dFood;
          result[j,i  ].fire-=dFire;
          result[j,i+1].food+=dFood;
          result[j,i+1].fire+=dFire;
        end;
        if j<SYS_HEIGHT-1 then begin
          calculateFlow(s[jm,i].food,s[j,i].food,s[j+1,i].food,s[jp,i].food,dFood,
                        s[jm,i].fire,s[j,i].fire,s[j+1,i].fire,s[jp,i].fire,dFire);
          result[j  ,i].food-=dFood;
          result[j  ,i].fire-=dFire;
          result[j+1,i].food+=dFood;
          result[j+1,i].fire+=dFire;
        end;
      end;
    end;
  end;

FUNCTION D_limited3rdOrderUpwind(CONST s:T_State):T_State;
  VAR i,j:longint;
      ip,im,jp,jm:longint;
      dFood,dFire: TmyFloat;

  PROCEDURE calculateFlow(CONST food_1,food0,food1,food2:TmyFloat; OUT foodFlow:TmyFloat;
                          CONST fire_1,fire0,fire1,fire2:TmyFloat; OUT fireFlow:TmyFloat);  //{$ifndef debugMode} inline; {$endif}
    VAR slopeRatio:TmyFloat;
    {$MACRO ON}
    {$define x0:=fire_1}
    {$define x1:=fire0}
    {$define x2:=fire1}
    {$define fluxLimitedUpwind:=begin
      if abs(x1-x0)<1E-6 then slopeRatio:=x1 else begin
        slopeRatio:=(x2-x1)/(x1-x0);
        slopeRatio:=max(0,min(1,min(slopeRatio,1/6+1/3*slopeRatio)));
        slopeRatio:=x1+slopeRatio*(x1-x0);
      end; end}
    begin
      fireFlow:=equationParameters.CONSUMER_MOVEMENT_FACTOR  *((food1-food0)*5/4+(food2-food_1)*-1/12);
      if fireFlow>0
      then begin
        {$define x0:=fire_1}
        {$define x1:=fire0}
        {$define x2:=fire1}
        fluxLimitedUpwind;
      end else begin
        {$define x0:=fire2}
        {$define x1:=fire1}
        {$define x2:=fire0}
        fluxLimitedUpwind;
      end;
      fireFlow*=slopeRatio;
      foodFlow:=equationParameters.FOOD_DIFFUSION_COEFFICIENT*((fire1-fire0)*5/4+(fire2-fire_1)*-1/12);
      if foodFlow>0
      then begin
        {$define x0:=food_1}
        {$define x1:=food0}
        {$define x2:=food1}
        fluxLimitedUpwind;
      end else begin
        {$define x0:=food2}
        {$define x1:=food1}
        {$define x2:=food0}
        fluxLimitedUpwind;
      end;
      foodFlow*=slopeRatio;
    end;

  begin
    for j:=0 to 1 do
    for i:=0 to SYS_WIDTH-1 do result[j,i]:=cellChemistry(s[j,i]);
    for j:=0 to SYS_HEIGHT-1 do begin

      jm:=j-1; if jm< 0          then jm:=0;
      jp:=j+2; if jp>=SYS_HEIGHT
      then jp:=SYS_HEIGHT-1
      else for i:=0 to SYS_WIDTH-1 do result[jp,i]:=cellChemistry(s[jp,i]);

      for i:=0 to SYS_WIDTH-1 do begin
        im:=i-1; if im< 0         then im:=0;
        ip:=i+2; if ip>=SYS_WIDTH then ip:=SYS_WIDTH-1;
        if i<SYS_WIDTH-1 then begin
          calculateFlow(s[j,im].food,s[j,i].food,s[j,i+1].food,s[j,ip].food,dFood,
                        s[j,im].fire,s[j,i].fire,s[j,i+1].fire,s[j,ip].fire,dFire);
          result[j,i  ].food-=dFood;
          result[j,i  ].fire-=dFire;
          result[j,i+1].food+=dFood;
          result[j,i+1].fire+=dFire;
        end;
        if j<SYS_HEIGHT-1 then begin
          calculateFlow(s[jm,i].food,s[j,i].food,s[j+1,i].food,s[jp,i].food,dFood,
                        s[jm,i].fire,s[j,i].fire,s[j+1,i].fire,s[jp,i].fire,dFire);
          result[j  ,i].food-=dFood;
          result[j  ,i].fire-=dFire;
          result[j+1,i].food+=dFood;
          result[j+1,i].fire+=dFire;
        end;
      end;
    end;
  end;

CONSTRUCTOR T_cellSystem.create;
  begin
    recommendedStepSize:=TIME_STEP_SIZE/100;
    D:=@D_limited3rdOrderUpwind;
  end;

OPERATOR *(CONST x:TmyFloat; CONST s:T_State):T_State; inline;
  VAR j,i:longint;
  begin
    for j:=0 to SYS_HEIGHT-1 do for i:=0 to SYS_WIDTH-1 do begin
      result[j,i].fire:=s[j,i].fire*x;
      result[j,i].food:=s[j,i].food*x;
    end;
  end;

OPERATOR +(CONST x,y:T_State):T_State; inline;
  VAR j,i:longint;
  begin
    for j:=0 to SYS_HEIGHT-1 do for i:=0 to SYS_WIDTH-1 do begin
      result[j,i].fire:=x[j,i].fire+y[j,i].fire;
      result[j,i].food:=x[j,i].food+y[j,i].food;
    end;
  end;

FUNCTION xpymz(CONST x,y:T_State; CONST z:TmyFloat):T_State; {$ifndef debugMode} inline; {$endif}
  VAR j,i:longint;
  begin
    for j:=0 to SYS_HEIGHT-1 do for i:=0 to SYS_WIDTH-1 do begin
      result[j,i].fire:=x[j,i].fire+y[j,i].fire*z;
      result[j,i].food:=x[j,i].food+y[j,i].food*z;
    end;
  end;

PROCEDURE pymz(VAR x:T_State; VAR y:T_State; CONST z:TmyFloat); {$ifndef debugMode} inline; {$endif}
  VAR j,i:longint;
  begin
    for j:=0 to SYS_HEIGHT-1 do for i:=0 to SYS_WIDTH-1 do begin
      x[j,i].fire+=y[j,i].fire*z;
      x[j,i].food+=y[j,i].food*z;
    end;
  end;

FUNCTION fifthOrderStepForward(CONST initialState:T_State; CONST D:T_derivativeFunction; CONST dt:TmyFloat):T_State;
  VAR F0, F1, F2, F3, F4: T_State;
  begin
    // 0.2296119676868942
    // 1.2388221524945009 -0.7550031854035194
    // 1.9868818065968332 -0.1725144944440256 -0.989877707376752
    //-1.4439873980538391  1.9004096550631115  0.8627573095507601 1.2858918672685673
    //-1.694135567449278   4.041995759072443  -1.808087874978392  0.42246268620353628 0.03776499715168942
    F0:=D(initialState);  result:=xpymz(initialState,F0, 0.2296119676868942*dt);
    F1:=D(result);        result:=xpymz(initialState,F0, 1.2388221524945009*dt); pymz(result,F1,-0.7550031854035194*dt);
    F2:=D(result);        result:=xpymz(initialState,F0, 1.9868818065968332*dt); pymz(result,F1,-0.1725144944440256*dt); pymz(result,F2,-0.989877707376752 *dt);
    F3:=D(result);        result:=xpymz(initialState,F0,-1.4439873980538391*dt); pymz(result,F1, 1.9004096550631115*dt); pymz(result,F2, 0.8627573095507601*dt); pymz(result,F3,1.2858918672685673 *dt);
    F4:=D(result);        result:=xpymz(initialState,F0,-1.694135567449278 *dt); pymz(result,F1, 4.041995759072443 *dt); pymz(result,F2,-1.808087874978392 *dt); pymz(result,F3,0.42246268620353628*dt); pymz(result,F4,0.03776499715168942 *dt);
  end;

{$ifdef enable_calibration}
FUNCTION trueErrorEstimation(CONST initialState, approximation:T_State; CONST D:T_derivativeFunction; CONST dt:TmyFloat):TmyFloat;
  VAR preciseEstimation:T_State;
      j,i:longint;
  begin
    preciseEstimation:=fifthOrderStepForward(initialState     ,D,dt/2);
    preciseEstimation:=fifthOrderStepForward(preciseEstimation,D,dt/2);
    result:=1E-10;
    for j:=0 to SYS_HEIGHT-1 do
    for i:=0 to SYS_WIDTH -1 do
    with preciseEstimation[j,i] do
      result:=max(max(abs(food-approximation[j,i].food),
                      abs(fire-approximation[j,i].fire)),result);
  end;

PROCEDURE T_persistedState.addCalibrationData(
    CONST method: T_timeIntegrationMethod; CONST steps, order: longint;
    CONST initialState, approximation: T_State; CONST D: T_derivativeFunction;
    CONST dt, estimatedError: TmyFloat);

  VAR actualError:TmyFloat;
      calibrationFileName: string;
  begin
    if not(calibration_open) then begin
      calibrationFileName:=ChangeFileExt(paramStr(0),'_calibration.txt');
      assign(calibration_handle,calibrationFileName);
      if fileExists(calibrationFileName)
      then append(calibration_handle)
      else rewrite(calibration_handle);
      calibration_open:=true;
    end;
    actualError:=trueErrorEstimation(initialState,approximation,D,dt);
    if method=MULTISTEP
    then writeln(calibration_handle, method,'(',steps,',',order,');',estimatedError,';',actualError)
    else writeln(calibration_handle, method,                     ';',estimatedError,';',actualError);
  end;
{$endif}

VAR F0,F1,F2,F3,F4,F5,approx1,approx2:T_State;
    f0_isReusable:boolean=false;
    multistep_reusable:byte=0;
    timeIntegrationMethod:T_timeIntegrationMethod=BOGACKI_SHAMPINE;
    multistep_steps:byte=3;
    multistep_order:byte=3;
PROCEDURE T_cellSystem.doMacroTimeStep(VAR user: T_userInteraction);
  VAR tol:TmyFloat=0.1;
  PROCEDURE updateByUser;
    VAR i,j,k,radius: longint;
        fireIncrement:TmyFloat;
    begin
      {$ifdef multithreading}
      enterCriticalSection(user.usercs);
      {$endif}
      if user.pointCount>0 then begin
        user.affirmLastPoint;
        for k:=0 to user.pointCount-1 do with user.points[k] do begin
          fireIncrement:=abs(user.igniteRate);
          radius:=round(3*fireIncrement);
          fireIncrement:=1/sqr(fireIncrement);
          for j:=-radius to radius do
          for i:=-radius to radius do
            modifyFire(ix+i,iy+j,exp((-i*i-j*j)*fireIncrement)*ir);
        end;
        user.points[0]:=user.points[user.pointCount-1];
        user.pointCount:=abs(user.igniting);
        f0_isReusable:=false;
        multistep_reusable:=0;
      end;
      {$ifdef multithreading}
      case user.snapshotState of
        ss_snapshot_queried: begin
          user.snapshot:=state;
          user.snapshotState:=ss_none;
        end;
        ss_set_posted: begin
          state:=user.snapshot;
          f0_isReusable:=false;
          multistep_reusable:=0;
          user.snapshotState:=ss_none;
        end;
      end;
      {$endif}
      if user.settingChanged then begin
        if timeIntegrationMethod<>user.timeIntegrationMethod
        then multistep_reusable:=0;
        multistep_order:=user.multistep_order;
        multistep_steps:=user.multistep_steps;
        timeIntegrationMethod:=user.timeIntegrationMethod;
        tol:=user.errorTolerance;
        equationParameters:=user.equation_parameters ;
        f0_isReusable:=false;
        recommendedStepSize:=TIME_STEP_SIZE*1E-3;

        case user.spatialDiscretization of
          FIRST_ORDER_UPWIND        : D:=@D_1stOrderUpwind;
          SECOND_ORDER_CENTRAL      : D:=@D_2ndOrderCentral;
          THIRD_ORDER_UPWIND        : D:=@D_3rdOrderUpwind;
          LIMITED_THIRD_ORDER_UPWIND: D:=@D_limited3rdOrderUpwind;
        end;
        user.settingChanged:=false;
      end;
      {$ifdef multithreading}
      leaveCriticalSection(user.usercs);
      {$endif}
    end;

  CONST TIME_OUT_TICKS=1000;
  VAR i,j,kSub:longint;
      dt,dtRest,
      delta,maxDelta: TmyFloat;
      timeOutAtTicks: qword;
      statistics:T_statistics;

      {$ifdef enable_calibration}
      calibrating:boolean=false;
      {$endif}

  PROCEDURE doMultistep(CONST steps,order:byte);
    TYPE T_Coeff=array[0..5,0..5] of TmyFloat;
    FUNCTION prepareCoeff(CONST t1,t2,t3,t4,t5:TmyFloat):T_Coeff;
      VAR i,j,k:longint;
          M:T_Coeff;
          fak:TmyFloat;
          t_pow:array[0..5,0..10] of TmyFloat;
          s_pow:array[1..10] of TmyFloat;
      begin
                        t_pow[0,1]:=0;
                        t_pow[1,1]:=t1;
        if steps>2 then t_pow[2,1]:=t2 else t_pow[2,1]:=0;
        if steps>3 then t_pow[3,1]:=t3 else t_pow[3,1]:=0;
        if steps>4 then t_pow[4,1]:=t4 else t_pow[4,1]:=0;
        if steps>5 then t_pow[5,1]:=t5 else t_pow[5,1]:=0;
        for j:=1 to 10 do s_pow[j]:=0;
        for i:=1 to 5 do begin
          t_pow[i,0]:=1;
          s_pow[1]+=t_pow[i,1];
          fak:=t_pow[i,1];
          for j:=2 to 5 do begin
            fak*=t_pow[i,1];
            t_pow[i,j]:=fak;
            s_pow[j]  +=fak;
          end;
          for j:=6 to order+order-2 do begin
            fak*=t_pow[i,1];
            s_pow[j]  +=fak;
          end;
        end;
        M[0,0]:=1; for j:=1 to order-1 do M[0,j]:=0;
        for i:=1 to order-1 do for j:=0 to order-1 do M[i,j]:=s_pow[i+j]*(j+1);

        result[0,0]:=1; for j:=1 to 5 do result[0,j]:=0;
        for i:=1 to order-1 do begin
          result[i,0]:=0;
          for j:=1 to steps-1 do result[i,j]:=t_pow[j,i];
          for j:=steps   to 5 do result[i,j]:=0;
        end;
        for i:=order to 5 do for j:=0 to 5 do result[i,j]:=0;

        //Solve via Gauss-Jordan
        for k:=0 to order-1 do begin
          fak:=1/M[k,k];
          M[k,k]:=1;
          for j:=k+1 to order-1 do M     [k,j]*=fak;
          for j:=0   to steps-1 do result[k,j]*=fak;
          for i:=0 to order-1 do if (i<>k) and (M[i,k]<>0) then begin
            fak:=-M[i,k];
            for j:=0 to order-1 do M     [i,j]+=fak*M     [k,j];
            for j:=0 to steps-1 do result[i,j]+=fak*result[k,j];
          end;
        end;
      end;
    VAR C: T_Coeff;
        i,j:longint;
        dtRel:TmyFloat;
        maxOscillation:TmyFloat=0;
    begin
      F5:=F4; prevDt[5]:=prevDt[4];
      F4:=F3; prevDt[4]:=prevDt[3];
      F3:=F2; prevDt[3]:=prevDt[2];
      F2:=F1; prevDt[2]:=prevDt[1];
      F1:=F0; prevDt[1]:=prevDt[0];
      F0:=D(state);
      inc(multistep_reusable); if multistep_reusable>5 then multistep_reusable:=6;

      if multistep_reusable<steps then begin
        prevDt[0]:=TIME_STEP_SIZE*1E-3;
        state:=fifthOrderStepForward(state,D,prevDt[0]); dt    :=prevDt[0];
        exit;
      end;
      C:=prepareCoeff((-prevDt[1]                                        )/prevDt[1],
                      (-prevDt[1]-prevDt[2]                              )/prevDt[1],
                      (-prevDt[1]-prevDt[2]-prevDt[3]                    )/prevDt[1],
                      (-prevDt[1]-prevDt[2]-prevDt[3]-prevDt[4]          )/prevDt[1],
                      (-prevDt[1]-prevDt[2]-prevDt[3]-prevDt[4]-prevDt[5])/prevDt[1]);
      //Extract highest order term for a priori error estimation:
      approx1:=                            (C[order-1,0])*F0;
                            pymz(approx1,F1,C[order-1,1]);
      if steps>2 then begin pymz(approx1,F2,C[order-1,2]);
      if steps>3 then begin pymz(approx1,F3,C[order-1,3]);
      if steps>4 then begin pymz(approx1,F4,C[order-1,4]);
      if steps>5 then       pymz(approx1,F5,C[order-1,5]); end; end; end;

      maxDelta:=1E-10;
      for j:=0 to SYS_HEIGHT-1 do for i:=0 to SYS_WIDTH-1 do with approx1[j,i] do begin
        maxDelta:=max(maxDelta,max(abs(food),abs(fire)));
      end;

      //Add simple oscillation detection:
      approx1:=       prevDt[1]*F0; pymz(approx1,F1,-prevDt[1]);
      pymz(approx1,F2,prevDt[1]);   pymz(approx1,F3,-prevDt[1]);
      pymz(approx1,F4,prevDt[1]);   pymz(approx1,F5,-prevDt[1]);
      for j:=0 to SYS_HEIGHT-1 do for i:=0 to SYS_WIDTH-1 do with approx1[j,i] do maxOscillation:=max(maxOscillation,max(abs(food),abs(fire)));
      maxOscillation*=0.25;

      recommendedStepSize:=max(TIME_STEP_SIZE*1E-4,
         prevDt[1]*min(power(tol/maxDelta      ,1/(order-1)),
                             tol/maxOscillation ));
      dt:=dtRest/ceil(dtRest/recommendedStepSize);
      dtRel:=dt/prevDt[1];

      if (recommendedStepSize<TIME_STEP_SIZE*1E-4) or
         (recommendedStepSize<prevDt[1]*1E-2) and (prevDt[1]>TIME_STEP_SIZE*1E-3) or
         isNan(dt) or isInfinite(dt) then begin
        //Full panic mode...
        for j:=0 to 5 do prevDt[j]:=recommendedStepSize;
        multistep_reusable:=0;
        dt:=0;
        exit;
      end;
      {$ifdef enable_calibration}
      if calibrating then approx2:=state;
      {$endif}
                            pymz(state,F0,(C[0,0] + (C[1,0] + (C[2,0] + (C[3,0] + (C[4,0] + C[5,0]*dtRel)*dtRel)*dtRel)*dtRel)*dtRel)*dt);
                            pymz(state,F1,(C[0,1] + (C[1,1] + (C[2,1] + (C[3,1] + (C[4,1] + C[5,1]*dtRel)*dtRel)*dtRel)*dtRel)*dtRel)*dt);
      if steps>2 then begin pymz(state,F2,(C[0,2] + (C[1,2] + (C[2,2] + (C[3,2] + (C[4,2] + C[5,2]*dtRel)*dtRel)*dtRel)*dtRel)*dtRel)*dt);
      if steps>3 then begin pymz(state,F3,(C[0,3] + (C[1,3] + (C[2,3] + (C[3,3] + (C[4,3] + C[5,3]*dtRel)*dtRel)*dtRel)*dtRel)*dtRel)*dt);
      if steps>4 then begin pymz(state,F4,(C[0,4] + (C[1,4] + (C[2,4] + (C[3,4] + (C[4,4] + C[5,4]*dtRel)*dtRel)*dtRel)*dtRel)*dtRel)*dt);
      if steps>5 then       pymz(state,F5,(C[0,5] + (C[1,5] + (C[2,5] + (C[3,5] + (C[4,5] + C[5,5]*dtRel)*dtRel)*dtRel)*dtRel)*dtRel)*dt); end; end; end;
      prevDt[0]:=dt;
      {$ifdef enable_calibration}
      if calibrating then persistedState.addCalibrationData(MULTISTEP,steps,order,approx2,state,D,dt,tol*power(dt/recommendedStepSize,order));
      {$endif}
    end;

  begin
    timeOutAtTicks:=GetTickCount64+TIME_OUT_TICKS;
    updateByUser;
    dtRest:=TIME_STEP_SIZE;
    kSub:=0;

    while (dtRest>0) and (user.recording or not((kSub>0) and (GetTickCount64>timeOutAtTicks))) do begin
      for j:=0 to SYS_HEIGHT-1 do for i:=0 to SYS_WIDTH-1 do with state[j,i] do begin
        if fire<0 then fire:=0;
        if food<0 then food:=0;
      end;
      {$ifdef enable_calibration}
      calibrating:=random<0.01; //about every 100th micro-step will be used for collecting calibration data
      {$endif}

      dt:=dtRest/ceil(dtRest/recommendedStepSize);
      case timeIntegrationMethod of
        MULTISTEP  : doMultistep(multistep_steps,multistep_order);
        BOGACKI_SHAMPINE: begin
          if not(f0_isReusable) then begin
            F0:=D(state);
            f0_isReusable:=true;
          end;
          approx2:=xpymz(state,F0,0.5 *dt);                                                    F1:=D(approx2);
          approx2:=xpymz(state,F1,0.75*dt);                                                    F2:=D(approx2);
          approx2:=xpymz(state,F0,2/9 *dt); pymz(approx2,F1,1/3 *dt); pymz(approx2,F2,4/9*dt); F3:=D(approx2);
          approx1:=xpymz(state,F0,7/24*dt); pymz(approx1,F1,0.25*dt); pymz(approx1,F2,1/3*dt); pymz(approx1,F3,0.125*dt);
        end;
        BSB_SSP_3: begin
          if not(f0_isReusable) then begin
            F0:=D(state);
            f0_isReusable:=true;
          end;
          //1/6
          //0   5/12
          //8/5 -3   12/5
          //3   -3   0    1
          approx2:=xpymz(state,F0, 1/6*dt);                                                      F1:=D(approx2);
                                  approx2:=xpymz(state  ,F1, 5/12*dt);                           F2:=D(approx2);
          approx2:=xpymz(state,F0, 8/5*dt); pymz(approx2,F1,-3   *dt); pymz(approx2,F2,12/5*dt); F3:=D(approx2);
          approx1:=xpymz(state,F0, 3  *dt); pymz(approx1,F1,-3   *dt); pymz(approx1,F3,     dt);
        end;
        BSB_SSP_4: begin
          if not(f0_isReusable) then begin
            F0:=D(state);
            f0_isReusable:=true;
          end;
          //1/6
          //0     1/3
          //0     -2/15 4/5
          //-5/16 3/2   -9/8 15/16
          //-2/15 3/5   1/30 1/3   1/6
          approx2:=xpymz(state,F0, 1/6 *dt);                          F1:=D(approx2);
          approx2:=xpymz(state,F1, 1/3 *dt);                          F2:=D(approx2);
          approx2:=xpymz(state,F1,-2/15*dt); pymz(approx2,F2,4/5*dt); F3:=D(approx2);
          approx2:=xpymz(state,F0,-5/16*dt); pymz(approx2,F1,3/2*dt); pymz(approx2,F2,-9/8 *dt); pymz(approx2,F3,15/16*dt); F4:=D(approx2);
          approx1:=xpymz(state,F0,-2/15*dt); pymz(approx1,F1,3/5*dt); pymz(approx1,F2, 1/30*dt); pymz(approx1,F3, 1/3 *dt); pymz(approx1,F4,1/6*dt);
        end;

        CASH_CARP       : begin
          if not(f0_isReusable) then begin
            F0:=D(state);
            f0_isReusable:=true;
          end;
          approx2:=xpymz(state,F0,   1/5    *dt);                                                                                                                              F1:=D(approx2);
          approx2:=xpymz(state,F0,   3/40   *dt); pymz(approx2,F1,  9/40 *dt);                                                                                                 F2:=D(approx2);
          approx2:=xpymz(state,F0,   3/10   *dt); pymz(approx2,F1, -9/10 *dt); pymz(approx2,F2,   6/5    *dt);                                                                 F3:=D(approx2);
          approx2:=xpymz(state,F0, -11/54   *dt); pymz(approx2,F1,  5/2  *dt); pymz(approx2,F2, -70/27   *dt); pymz(approx2,F3,   35/27    *dt);                               F4:=D(approx2);
          approx2:=xpymz(state,F0,1631/55296*dt); pymz(approx2,F1,175/512*dt); pymz(approx2,F2, 575/13824*dt); pymz(approx2,F3,44275/110592*dt); pymz(approx2,F4,253/4096*dt); F5:=D(approx2);
          approx2:=xpymz(state,F0,  37/378  *dt);                              pymz(approx2,F2, 250/621  *dt); pymz(approx2,F3,  125/594   *dt); pymz(approx2,F5,512/1771*dt);
          approx1:=xpymz(state,F0,2825/27648*dt); pymz(approx1,F2,18575/48384*dt); pymz(approx1,F3,13525/55296 *dt); pymz(approx1,F4,277/14336*dt); pymz(approx1,F5,1/4*dt);
        end;
        else begin
          //HEUN_EULER
          if not(f0_isReusable) then F0:=D(state);
          f0_isReusable:=true;
          approx1:=xpymz(state,F0,dt); //euler step
          approx2:=xpymz(state,F0,dt*0.5);
          F1:=D(approx1);
          pymz(state,F1,dt*0.5);
        end;
      end;

      if timeIntegrationMethod=MULTISTEP then begin
        dtRest-=dt;
        inc(kSub);
        {$ifdef multithreading}
        if user.settingChanged or (user.snapshotState<>ss_none) then updateByUser;
        {$endif}
      end else begin
        maxDelta:=1E-10;
        for j:=0 to SYS_HEIGHT-1 do for i:=0 to SYS_WIDTH-1 do with approx2[j,i] do begin
          delta:=abs(approx1[j,i].food-food); if delta>maxDelta then maxDelta:=delta;
          delta:=abs(approx1[j,i].fire-fire); if delta>maxDelta then maxDelta:=delta;
        end;
        {$ifdef enable_calibration}
        if calibrating then persistedState.addCalibrationData(timeIntegrationMethod,0,0,state,approx2,D,dt,maxDelta);
        {$endif}
        case timeIntegrationMethod of
          BOGACKI_SHAMPINE,
          BSB_SSP_3 : recommendedStepSize:=dt*power(tol/maxDelta,1/3);
          CASH_CARP : recommendedStepSize:=dt*power(tol/maxDelta,1/5);
          BSB_SSP_4 : recommendedStepSize:=dt*power(tol/maxDelta,1/4);
          HEUN_EULER: recommendedStepSize:=dt*sqrt (tol/maxDelta);
        end;
        if ((recommendedStepSize*1.01>=dt) and not(user.settingChanged)) then begin
          state:=approx2;
          dtRest-=dt;
          inc(kSub);
          case timeIntegrationMethod of
            BSB_SSP_3,
            BOGACKI_SHAMPINE: F0:=F3;
            BSB_SSP_4: F0:=F4;
            else  f0_isReusable:=false;
          end;
        end else begin
          {$ifdef multithreading}
          if user.settingChanged or (user.snapshotState<>ss_none) then updateByUser;
          {$endif}
          {$ifdef debugMode}
          writeln('Step: ',timeStepIndex,'/',kSub,' dt=',dt:0:5,' dt''=',recommendedStepSize:0:5,' -- rejected ');
          {$endif}
        end;
      end;
    end;
    {$ifdef debugMode}
    writeln('Step: ',timeStepIndex,'/',kSub,' dt=',dt:0:5,' dt''=',recommendedStepSize:0:5,' (',timeIntegrationMethod,')');
    {$endif}

    statistics.avgFire:=0;
    statistics.avgFood:=0;
    statistics.minFire:=FIRE_CAP;
    statistics.minFood:=FOOD_CAP;
    statistics.maxFire:=0;
    statistics.maxFood:=0;
    for j:=0 to SYS_HEIGHT-1 do for i:=0 to SYS_WIDTH-1 do with state[j,i] do begin
      if food<statistics.minFood then statistics.minFood:=food;
      if food>statistics.maxFood then statistics.maxFood:=food;
      statistics.avgFood+=food;

      if fire<statistics.minFire then statistics.minFire:=fire;
      if fire>statistics.maxFire then statistics.maxFire:=fire;
      statistics.avgFire+=fire;
    end;
    statistics.avgFood*=1/(SYS_WIDTH*SYS_HEIGHT);
    statistics.avgFire*=1/(SYS_WIDTH*SYS_HEIGHT);
    user.logStatistics(statistics);
    inc(timeStepIndex);
  end;

PROCEDURE T_cellSystem.getPicture(VAR rgbPicture:T_rgbPicture);
  CONST WHITE_LEVEL =1.5;
  VAR r,b:TmyFloat;
  VAR i,j:longint;
  begin
    enterCriticalSection(rgbPicture.cs);
    for j:=0 to SYS_HEIGHT-1 do for i:=0 to SYS_WIDTH-1 do begin
      r:=state[j,i].fire/WHITE_LEVEL;
      b:=state[j,i].food/WHITE_LEVEL;
      rgbPicture.setValue(i,j,r,b);
    end;
    rgbPicture.invalidateBitmap;
    leaveCriticalSection(rgbPicture.cs);
  end;

INITIALIZATION
  SetExceptionMask([ exInvalidOp,  exDenormalized,  exZeroDivide,  exOverflow,  exUnderflow,  exPrecision]);
  persistedState.create;

FINALIZATION
  persistedState.destroy;

end.

