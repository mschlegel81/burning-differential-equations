UNIT burnPhysics;
{$mode objfpc}{$H+}
INTERFACE
USES Graphics;
CONST
  GROWTH_THRESHOLD =1;
  DIE_THRESHOLD    =0.25;

  SPATIAL_SCALE=1;

  CONSUMER_MOVEMENT_FACTOR  =1*SPATIAL_SCALE;
  FOOD_DIFFUSION_COEFFICIENT=1*SPATIAL_SCALE;

  GROWTH_SPEED              =0.5;
  DIE_SPEED                 =1;
  CONSUMPTION_SPEED         =0.01;
  GROW_BACK_FACTOR          =0.01;

  EQUILIBRIUM_RATIO=GROW_BACK_FACTOR/CONSUMPTION_SPEED;
  FOOD_CAP    =50;
  CONSUMER_CAP=50;

  TIME_STEP_SIZE =0.5;
  FEEDBACK_LENGTH=50;

  ERROR_TOLERANCE=0.0001;
  MAX_TEMPORAL_REFINEMENT_LEVEL=16;

TYPE
  T_value=record fire,food:double; end;

  P_cell=^T_cell;

  { T_cell }

  T_cell=object
    private
      y0,
      value:T_value;

      neighbor:array[0..3] of P_cell;

      errorEstimate:double;
      temporalRefinementLevel:byte;

      regrow:double;
      inflowFromSlowerNeighbors:T_value;
      stageFlow:array[0..2] of T_value;

    public
      CONSTRUCTOR create(CONST leftNeighbor,topNeighbor:P_cell);

      PROCEDURE alignNeighborTimeLevel(CONST targetLevel:byte);
      PROCEDURE calculateDelta(CONST stage,stageOnLevelAbove:byte);
      PROCEDURE applyDelta(CONST stage:byte; CONST dt:double);
      PROCEDURE resetAllDeltas;
      FUNCTION cellColor:TColor;
  end;

  { T_cellSystem }

  T_cellSystem=object
    private
      Cells:P_cell;
      regrow:array [0..FEEDBACK_LENGTH-1] of PDouble;

      systemWidth,systemHeight,systemCellCount:longint;
    public
      CONSTRUCTOR create(CONST width,height:longint);
      DESTRUCTOR destroy;

      PROCEDURE doMacroTimeStep;
      PROCEDURE draw(Canvas:TCanvas);
  end;

CONST zeroValue:T_value=(fire:0; food:0);

IMPLEMENTATION

OPERATOR +(CONST x,y:T_value):T_value;
  begin
    result.fire:=x.fire+y.fire;
    result.food:=x.food+y.food;
  end;

OPERATOR *(CONST x:T_value; CONST y:double):T_value;
  begin
    result.fire:=x.fire*y;
    result.food:=x.food*y;
  end;

{ T_cellSystem }

CONSTRUCTOR T_cellSystem.create(CONST width, height: longint);
  VAR i,j,k:longint;
      Left,up:P_cell;
  begin
    systemWidth    :=width;
    systemHeight   :=height;
    systemCellCount:=width*height;

    getMem(Cells,sizeOf(T_cell)*systemCellCount);
    k:=0;
    for j:=0 to height-1 do for i:=0 to width-1 do begin
      if i>0 then Left:=Cells+(k-1    ) else Left:=nil;
      if j>0 then up  :=Cells+(k-width) else up  :=nil;
      Cells[k].create(Left,up);

      Cells[k].value.fire:=j/(height-1);
      Cells[k].value.food:=i/(width-1);

      inc(k);
    end;

    for k:=0 to FEEDBACK_LENGTH-1 do begin
      getMem(regrow[k],sizeOf(double)*systemCellCount);
      for i:=0 to systemCellCount-1 do regrow[k][i]:=Cells[i].value.food*GROW_BACK_FACTOR;
    end;

    //cells[0].value*=2.0;

  end;

DESTRUCTOR T_cellSystem.destroy;
begin

end;

PROCEDURE T_cellSystem.doMacroTimeStep;

  VAR maxTimeLevel:longint=0;
  PROCEDURE doMicroTimestep(CONST level,stageAbove:byte; CONST dt:double);
    VAR k:longint;
    begin
      //Stage 0:
      for k:=0 to systemCellCount-1 do if Cells[k].temporalRefinementLevel=level then Cells[k].resetAllDeltas;
      for k:=0 to systemCellCount-1 do if Cells[k].temporalRefinementLevel=level+1 then Cells[k].inflowFromSlowerNeighbors:=zeroValue;
      for k:=0 to systemCellCount-1 do if Cells[k].temporalRefinementLevel=level then Cells[k].calculateDelta(0,stageAbove);
      if level<maxTimeLevel then doMicroTimestep(level+1,0,dt*0.5);
      for k:=0 to systemCellCount-1 do if Cells[k].temporalRefinementLevel=level then Cells[k].applyDelta(0,dt);

      //Stage 1:
      for k:=0 to systemCellCount-1 do if Cells[k].temporalRefinementLevel=level then Cells[k].resetAllDeltas;
      for k:=0 to systemCellCount-1 do if Cells[k].temporalRefinementLevel=level+1 then Cells[k].inflowFromSlowerNeighbors:=zeroValue;
      for k:=0 to systemCellCount-1 do if Cells[k].temporalRefinementLevel=level then Cells[k].calculateDelta(1,stageAbove);
      if level<maxTimeLevel then doMicroTimestep(level+1,1,dt*0.5);
      for k:=0 to systemCellCount-1 do if Cells[k].temporalRefinementLevel=level then Cells[k].applyDelta(1,dt);

      //Stage 2:
      for k:=0 to systemCellCount-1 do if Cells[k].temporalRefinementLevel=level then Cells[k].calculateDelta(2,stageAbove);
      for k:=0 to systemCellCount-1 do if Cells[k].temporalRefinementLevel=level then Cells[k].applyDelta(2,dt);
    end;

  VAR temp:PDouble;
      targetRefinementLevel:longint;
      error:double;
      i:longint;

      refinements:array[0..MAX_TEMPORAL_REFINEMENT_LEVEL] of longint;
      initialState:array of T_value;
      allRefinementLevelsEqual:boolean;
      loopCount:longint=0;
  begin
    setLength(initialState,systemCellCount);
    for i:=0 to systemCellCount-1 do initialState[i]:=Cells[i].value;

    repeat
      for i:=0 to length(refinements)-1 do refinements[i]:=0;
      maxTimeLevel:=0;
      allRefinementLevelsEqual:=true;

      //Initialization per cell...
      for i:=0 to systemCellCount-1 do begin
        //Regrowth
        Cells[i].regrow:=regrow[0][i];
        //Initial value
        Cells[i].y0   :=initialState[i];
        Cells[i].value:=initialState[i];
        Cells[i].inflowFromSlowerNeighbors:=zeroValue;

        //Temporal refinement based on error estimate
        //Since we are using a 2nd Order time integration method, the error is proportional to the square of the time step size.
        //Decreasing the temporal refinement by 1 means doubling of the time step and thus quadrupling of the error and vice versa.
        targetRefinementLevel:=Cells[i].temporalRefinementLevel;
        error:=Cells[i].errorEstimate;
        if error<0.75*ERROR_TOLERANCE then while (error<0.125*ERROR_TOLERANCE) and (targetRefinementLevel>0) and (loopCount=0) do begin
          targetRefinementLevel-=1; error*=4;
        end else while (error>ERROR_TOLERANCE) and (targetRefinementLevel<MAX_TEMPORAL_REFINEMENT_LEVEL) do begin
          targetRefinementLevel+=1; error*=0.25;
        end;
        Cells[i].errorEstimate:=0;
        allRefinementLevelsEqual:=allRefinementLevelsEqual and (Cells[i].temporalRefinementLevel=targetRefinementLevel);
        Cells[i].temporalRefinementLevel:=targetRefinementLevel;
        if targetRefinementLevel>maxTimeLevel then maxTimeLevel:=targetRefinementLevel;
      end;

      //Align temporal refinement
      for i:=0 to systemCellCount-1 do Cells[i].alignNeighborTimeLevel(Cells[i].temporalRefinementLevel);
      //for i:=0 to systemCellCount-1 do cells[i].temporalRefinementLevel:=maxTimeLevel;

      for i:=0 to systemCellCount-1 do inc(refinements[Cells[i].temporalRefinementLevel]);

      write(stdErr,'TLevels:');
      for i:=0 to maxTimeLevel do write(stdErr,' ',refinements[i]);
      writeln(stdErr,'');

      doMicroTimestep(0,0,TIME_STEP_SIZE);
      error:=0;
      for i:=0 to systemCellCount do if Cells[i].errorEstimate>error then error:=Cells[i].errorEstimate;

      writeln(stdErr,'Error: ',error/ERROR_TOLERANCE,' * tolerance');
      inc(loopCount);
    until (error<ERROR_TOLERANCE) or (loopCount>=2);
    writeln(stdErr,'-------------------------------------------------');
    //Shift and update regrowth
    temp:=regrow[0];
    for i:=0 to FEEDBACK_LENGTH-2 do regrow[i]:=regrow[i+1];
    regrow[FEEDBACK_LENGTH-1]:=temp;
    for i:=0 to systemCellCount-1 do temp[i]:=Cells[i].value.food*GROW_BACK_FACTOR;
  end;

PROCEDURE T_cellSystem.draw(Canvas: TCanvas);
  VAR i,j:longint;
      k:longint=0;
  begin
    k:=0;
    for j:=0 to systemHeight-1 do for i:=0 to systemWidth-1 do begin
      Canvas.Pixels[i,j]:=Cells[k].cellColor;
      inc(k);
    end;
  end;

{ T_cell }

CONSTRUCTOR T_cell.create(CONST leftNeighbor, topNeighbor: P_cell);
  begin
    value.food:=0.4+0.01*random;
    value.fire:=value.food*EQUILIBRIUM_RATIO;
    neighbor[0]:=leftNeighbor;
    neighbor[1]:=nil;           if leftNeighbor<>nil then leftNeighbor^.neighbor[1]:=@self;
    neighbor[2]:=topNeighbor;
    neighbor[3]:=nil;           if topNeighbor <>nil then topNeighbor ^.neighbor[3]:=@self;
    errorEstimate:=0;
    regrow:=0;
    temporalRefinementLevel:=1;
  end;

PROCEDURE T_cell.alignNeighborTimeLevel(CONST targetLevel: byte);
  VAR n:integer;
  begin
    temporalRefinementLevel:=targetLevel;
    for n:=0 to 3 do
    if (neighbor[n]<>nil) and
       (neighbor[n]^.temporalRefinementLevel<targetLevel-1)
    then neighbor[n]^.alignNeighborTimeLevel(targetLevel-1);
  end;

PROCEDURE T_cell.calculateDelta(CONST stage, stageOnLevelAbove: byte);
  VAR nv:T_value;
      transported:double;
      toNeighbor:T_value;
      n:integer;
  begin
    for n:=0 to 3 do if neighbor[n]<>nil then begin
      toNeighbor:=zeroValue;
      nv:=neighbor[n]^.value;
      if nv.food>value.food then begin
        transported:=value.fire*(nv.food-value.food)*CONSUMER_MOVEMENT_FACTOR;
        toNeighbor.fire      +=transported;
        stageFlow[stage].fire-=transported;
      end;
      if nv.fire<value.fire then begin
        transported:=value.food*(value.fire-nv.fire)*FOOD_DIFFUSION_COEFFICIENT;
        toNeighbor.food      +=transported;
        stageFlow[stage].food-=transported;
      end;

      if   neighbor[n]^.temporalRefinementLevel=temporalRefinementLevel then neighbor[n]^.stageFlow[stage]+=toNeighbor
      else if neighbor[n]^.temporalRefinementLevel>temporalRefinementLevel
      then neighbor[n]^.inflowFromSlowerNeighbors   +=toNeighbor
      else neighbor[n]^.stageFlow[stageOnLevelAbove]+=toNeighbor*(1/3);
    end;

    stageFlow[stage].food-=value.fire*CONSUMPTION_SPEED;
    //if value.food>FOOD_CAP     then stageFlow[stage].food-=(value.food-FOOD_CAP);
    //if value.fire>CONSUMER_CAP then stageFlow[stage].fire-=(value.fire-CONSUMER_CAP);
    //
    if      value.food<DIE_THRESHOLD    then stageFlow[stage].fire-=(DIE_THRESHOLD-value.food   )*DIE_SPEED   *value.fire
    else if value.food>GROWTH_THRESHOLD then stageFlow[stage].fire+=(value.food-GROWTH_THRESHOLD)*GROWTH_SPEED*value.fire;
  end;

PROCEDURE T_cell.applyDelta(CONST stage: byte; CONST dt: double);
  PROCEDURE updateErrorEstimate;
    VAR e:double;
    begin
      e:=value.fire-y0.fire;
      if e<0 then e:=-e;
      if e>errorEstimate then errorEstimate:=e;

      e:=value.food-y0.food;
      if e<0 then e:=-e;
      if e>errorEstimate then errorEstimate:=e;
    end;

  begin
    case stage of
      0: value+=stageFlow[0]*(dt*0.5);
      1: value+=stageFlow[1]*(dt*0.5);
      2: begin
        y0+=(stageFlow[0]+stageFlow[1]+stageFlow[2])*(dt*(1/3));
        updateErrorEstimate;
        value:=y0;
      end;
    end;
    if value.food<0 then value.food:=0;
    if value.fire<0 then value.fire:=1E-3;
  end;

PROCEDURE T_cell.resetAllDeltas;
  begin
    y0:=value;
    stageFlow[0]:=inflowFromSlowerNeighbors;
    stageFlow[0].food+=regrow;

    stageFlow[1]:=stageFlow[0];
    stageFlow[2]:=stageFlow[0];
  end;

FUNCTION T_cell.cellColor: TColor;
  CONST WHITE_LEVEL =GROWTH_THRESHOLD*1.5;
        CWHITE_LEVEL=WHITE_LEVEL*EQUILIBRIUM_RATIO;
  VAR r,g,b:double;
  begin
    r:=value.fire/CWHITE_LEVEL;
    b:=value.food/WHITE_LEVEL;
    g:=(r+b)*0.5;
    if r>1 then r:=1 else if r<0 then r:=0;
    if g>1 then g:=1 else if g<0 then g:=0;
    if b>1 then b:=1 else if b<0 then b:=0;

    result:=
    round(r*255) or
   (round(g*255) shl 8) or
   (round(b*255) shl 16);
  end;

end.

