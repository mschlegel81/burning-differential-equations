UNIT basicGraphics;

{$mode objfpc}{$H+}

INTERFACE
USES ExtCtrls, Classes, BGRABitmapTypes, BGRABitmap;
CONST
  SYS_WIDTH =340;
  SYS_HEIGHT=180;
TYPE
  T_rgbColor=array[0..2] of byte;
  T_frameData=array[0..SYS_HEIGHT-1,0..SYS_WIDTH-1] of T_rgbColor;
  T_packedFrameData=array[0..SYS_HEIGHT-1,0..(SYS_WIDTH*9) div 4-1] of byte;
  T_upsampledDataRow=array[-11-3*4..4*(SYS_WIDTH+2)+11,0..1] of longint;

CONST
  FRAME_DATA_SIZE=sizeOf(T_frameData);
  PACKED_FRAME_DATA_SIZE=sizeOf(T_packedFrameData);
TYPE
  P_rgbPicture=^T_rgbPicture;

  { T_rgbPicture }

  T_rgbPicture=object
    private
      followedBy:P_rgbPicture;
      Pixels:T_frameData;
      Bitmap:TBGRABitmap;
    public
      cs:TRTLCriticalSection;
      CONSTRUCTOR create();
      DESTRUCTOR destroy;

      PROCEDURE copyToImage(VAR destImage:TImage);
      PROCEDURE setValue(CONST x,y:longint; CONST redLevel,blueLevel:double);
      PROCEDURE writeToStream(stream:TFileStream; CONST picIndex:longint);
      PROCEDURE loadFromStream(stream:TFileStream; CONST picIndex:longint);
      PROCEDURE exportToPng(CONST fileName:string);
      FUNCTION ensureBitmap(CONST resized:boolean=false):boolean;
      PROCEDURE invalidateBitmap;
  end;

  F_progressFunction=FUNCTION(CONST progress:double):boolean of object;

  P_recorder=^T_recorder;
  P_animation=^T_animation;
  T_recorder=object
    private
      reading:boolean;
      fileStream:TFileStream;
      frameCount:longint;
    public
      frameIndex:longint;
      CONSTRUCTOR createForRecording(CONST filename_:string);
      CONSTRUCTOR createForPlayback(CONST filename_:string);
      PROCEDURE restartRecording;
      PROCEDURE appendFrame(CONST pic:P_rgbPicture);
      {$ifdef multithreading}
      PROCEDURE getNextFrame(CONST scalingQueue:P_animation);
      {$else}
      PROCEDURE getNextFrame(VAR image:TImage; CONST Scaled:boolean);
      {$endif}
      PROPERTY frames:longint read frameCount;
      PROCEDURE exportToPngs(CONST pathPrefix:string; CONST resized:boolean; CONST pf:F_progressFunction);
      DESTRUCTOR destroy;
      FUNCTION recorderState:string;
  end;

  T_animation=object
    private
      section:TRTLCriticalSection;
      firstFrame,lastFrame:P_rgbPicture;
      frameCount:longint;
    public
      CONSTRUCTOR create;
      DESTRUCTOR destroy;
      FUNCTION render(VAR destImage:TImage; CONST recorder:P_recorder):boolean;
      FUNCTION getNext(CONST unlink:boolean=true):P_rgbPicture;
      FUNCTION dropFrame:longint;
      FUNCTION addFrame(CONST frame:P_rgbPicture):longint;
      PROPERTY framesCached:longint read frameCount;
  end;

PROCEDURE exportToPngsInBackground(CONST recorder:P_recorder; CONST resized:boolean; CONST pathPrefix:string);
IMPLEMENTATION
USES sysutils, Graphics, IntfGraphics, GraphType, math;

TYPE T_exportToPngTask=record
       recorder:P_recorder;
       resized:boolean;
       pathPrefix:string;
     end;
     P_exportToPngTask=^T_exportToPngTask;
VAR exportThreadsRunning:longint=0;
FUNCTION exportThread(p:pointer):ptrint;
  begin
    with P_exportToPngTask(p)^ do begin
      recorder^.exportToPngs(pathPrefix,resized,nil);
      dispose(recorder,destroy);
    end;
    freeMem(p,sizeOf(T_exportToPngTask));
    interlockedDecrement(exportThreadsRunning);
    result:=0;
  end;

PROCEDURE exportToPngsInBackground(CONST recorder:P_recorder; CONST resized:boolean; CONST pathPrefix:string);
  VAR task:P_exportToPngTask;
  begin
    getMem(task,sizeOf(T_exportToPngTask));
    task^.recorder  :=recorder;
    task^.pathPrefix:=pathPrefix;
    task^.resized   :=resized;
    interLockedIncrement(exportThreadsRunning);
    beginThread(@exportThread,task);
  end;

{ T_recorder }

CONSTRUCTOR T_recorder.createForRecording(CONST filename_: string);
  begin
    fileStream:=TFileStream.create(filename_,fmCreate or fmOpenWrite);
    frameCount:=0;
    frameIndex:=0;
    reading:=false;
  end;

CONSTRUCTOR T_recorder.createForPlayback(CONST filename_: string);
  begin
    fileStream:=TFileStream.create(filename_,fmOpenRead);
    frameCount:=fileStream.size div PACKED_FRAME_DATA_SIZE;
    frameIndex:=0;
    reading:=true;
  end;

PROCEDURE T_recorder.restartRecording;
  begin
    if not(reading) then frameCount:=0;
  end;

PROCEDURE T_recorder.appendFrame(CONST pic: P_rgbPicture);
  begin
    if reading then exit;
    pic^.writeToStream(fileStream,frameCount); inc(frameCount);
  end;

{$ifdef multithreading}
PROCEDURE T_recorder.getNextFrame(CONST scalingQueue:P_animation);
  VAR frame:P_rgbPicture;
  begin
    if not(reading) then exit;
    if frameIndex>=frameCount then exit;
    if frameIndex<0 then exit;
    new(frame,create());
    frame^.loadFromStream(fileStream,frameIndex);
    scalingQueue^.addFrame(frame);
    inc(frameIndex);
    if frameIndex>=frameCount then frameIndex:=frameCount-1;
  end;
{$else}
PROCEDURE T_recorder.getNextFrame(VAR image:TImage; CONST Scaled:boolean);
  VAR frame:T_rgbPicture;
  begin
    if not(reading) then exit;
    if frameIndex>=frameCount then exit;
    if frameIndex<0 then exit;
    frame.create;
    frame.loadFromStream(fileStream,frameIndex);
    frame.ensureBitmap(Scaled);
    frame.copyToImage(image);
    frame.destroy;
    inc(frameIndex);
    if frameIndex>=frameCount then frameIndex:=frameCount-1;
  end;
{$endif}

DESTRUCTOR T_recorder.destroy;
  begin
    FreeAndNil(fileStream);
  end;

FUNCTION T_recorder.recorderState: string;
  begin
    if not(reading)
    then result:='Recording: frame '
    else result:='Playing: frame '+intToStr(frameIndex)+'/';
    result+=intToStr(frameCount)+' ('+floatToStrF(PACKED_FRAME_DATA_SIZE*frameCount*9.5367431640625E-7,ffFixed,2,2)+'MiB)';
  end;

PROCEDURE T_recorder.exportToPngs(CONST pathPrefix: string; CONST resized:boolean; CONST pf: F_progressFunction);
  VAR i:longint;
      digitsRequired:longint=1;
      pngName:string;
      resume:boolean;
      frame:T_rgbPicture;
  begin

    frame.create;
    i:=frameCount-1;
    while i>=10 do begin
      inc(digitsRequired);
      i:=i div 10;
    end;
    for i:=0 to frameCount-1 do begin
      frame.loadFromStream(fileStream,i);
      pngName:=intToStr(i);
      while length(pngName)<digitsRequired do pngName:='0'+pngName;
      pngName:=ChangeFileExt(pathPrefix,'_'+pngName+'.png');
      frame.ensureBitmap(resized);
      frame.exportToPng(pngName);
      if pf<>nil then begin
        resume:=pf((i+1)/frameCount);
        if not(resume) then exit;
      end;
    end;
    frame.destroy;
  end;

{ T_animation }

CONSTRUCTOR T_animation.create;
  begin
    initCriticalSection(section);
    frameCount:=0;
    firstFrame:=nil;
    lastFrame:=nil;
  end;

DESTRUCTOR T_animation.destroy;
  begin
    enterCriticalSection(section);
    while frameCount>0 do dropFrame;
    leaveCriticalSection(section);
    doneCriticalSection(section);
  end;

FUNCTION T_animation.render(VAR destImage: TImage; CONST recorder:P_recorder): boolean;
  begin
    if (frameCount>0) and (firstFrame<>nil) then begin
      if recorder<>nil then recorder^.appendFrame(firstFrame);
      firstFrame^.copyToImage(destImage);
      result:=true;
    end else result:=false;
  end;

FUNCTION T_animation.getNext(CONST unlink:boolean=true):P_rgbPicture;
  begin
    enterCriticalSection(section);
    if (frameCount>0) and (firstFrame<>nil) then begin
      result:=firstFrame;
      if unlink then begin
        firstFrame:=firstFrame^.followedBy;
        result^.followedBy:=nil;
        dec(frameCount);
      end;
    end else result:=nil;
    leaveCriticalSection(section);
  end;

FUNCTION T_animation.dropFrame: longint;
  VAR dropped:P_rgbPicture=nil;
  begin
    enterCriticalSection(section);
    if (frameCount>0) and (firstFrame<>nil) then begin
      dropped:=firstFrame;
      firstFrame:=firstFrame^.followedBy;
      dec(frameCount);
      dispose(dropped,destroy);
    end;
    result:=frameCount;
    leaveCriticalSection(section);
  end;

FUNCTION T_animation.addFrame(CONST frame: P_rgbPicture): longint;
  begin
    enterCriticalSection(section);
    if firstFrame=nil then begin
      firstFrame:=frame;
      frameCount:=0;
    end else
      lastFrame^.followedBy:=frame;
    lastFrame:=frame;
    inc(frameCount);
    result:=frameCount;
    leaveCriticalSection(section);
  end;

{ T_rgbPicture }

CONSTRUCTOR T_rgbPicture.create;
  begin
    followedBy:=nil;
    Bitmap:=nil;
    InitCriticalSection(cs);
  end;

DESTRUCTOR T_rgbPicture.destroy;
  begin
    EnterCriticalSection(cs);
    if Bitmap<>nil then FreeAndNil(Bitmap);
    LeaveCriticalSection(cs);
    DoneCriticalSection(cs);
  end;

PROCEDURE halfTransform1(CONST c:T_rgbColor; OUT trueRed,trueBlue:word); inline;
  begin
    trueRed :=c[0]+((c[1] and 15 ) shl 8);
    trueBlue:=c[2]+((c[1] and 240) shl 4);
  end;

FUNCTION halfTransform2(trueRed,trueBlue:longint):TBGRAPixel; {$ifndef debugMode} inline; {$endif}
  VAR trueGreen:longint;
  begin
    trueGreen:=(trueRed+trueBlue) shr 1;
    if trueRed  >255 then trueRed  :=255;
    if trueGreen>255 then trueGreen:=255;
    if trueBlue >255 then trueBlue :=255;
    result.FromRGB(trueRed,trueGreen,trueBlue);
  end;

FUNCTION transformColor(CONST c:T_rgbColor):TBGRAPixel;
  VAR trueRed,trueBlue:word;
  begin
    halfTransform1(c,trueRed,trueBlue);
    result:=halfTransform2(trueRed,trueBlue);
  end;

PROCEDURE T_rgbPicture.setValue(CONST x, y: longint; CONST redLevel, blueLevel: double);
  VAR RED,BLUE:word;
      col:T_rgbColor;
  begin
    if (redLevel >2) or isNan(redLevel ) then RED :=511 else if redLevel <0 then RED :=0 else RED :=round(redLevel *255.5);
    if (blueLevel>2) or isNan(blueLevel) then BLUE:=511 else if blueLevel<0 then BLUE:=0 else BLUE:=round(blueLevel*255.5);

    col[0]:=RED  and 255;
    col[1]:=((RED  shr 8) and 15) or
            ((BLUE shr 4) and 240);
    assert(col[1] and 17 = col[1]);

    col[2]:=BLUE and 255;
    Pixels[y,x]:=col;
  end;

FUNCTION pack(CONST Pixels:T_frameData):T_packedFrameData;
  VAR y,x,k:longint;
      greens:byte;
  begin
    for y:=0 to SYS_HEIGHT-1 do begin
      x:=0;
      k:=0;
      while x<SYS_WIDTH do begin
        result[y,k]:=Pixels[y,x,0]; inc(k);
        greens     :=Pixels[y,x,1];
        result[y,k]:=Pixels[y,x,2]; inc(k); inc(x);
        result[y,k]:=Pixels[y,x,0]; inc(k);
        greens    :=(Pixels[y,x,1] shl 1) or greens;
        result[y,k]:=Pixels[y,x,2]; inc(k); inc(x);
        result[y,k]:=Pixels[y,x,0]; inc(k);
        greens    :=(Pixels[y,x,1] shl 2) or greens;
        result[y,k]:=Pixels[y,x,2]; inc(k); inc(x);
        result[y,k]:=Pixels[y,x,0]; inc(k);
        greens    :=(Pixels[y,x,1] shl 3) or greens;
        result[y,k]:=Pixels[y,x,2]; inc(k); inc(x);
        result[y,k]:=greens;        inc(k);
      end;
    end;
  end;

FUNCTION unpack(CONST packed_data:T_packedFrameData):T_frameData;
  VAR y,x,k:longint;
      greens:byte;
  begin
    for y:=0 to SYS_HEIGHT-1 do begin
      x:=0;
      k:=0;
      while x<SYS_WIDTH do begin
        result[y,x,0]:=packed_data[y,k]; inc(k);
        result[y,x,2]:=packed_data[y,k]; inc(k); inc(x);
        result[y,x,0]:=packed_data[y,k]; inc(k);
        result[y,x,2]:=packed_data[y,k]; inc(k); inc(x);
        result[y,x,0]:=packed_data[y,k]; inc(k);
        result[y,x,2]:=packed_data[y,k]; inc(k); inc(x);
        result[y,x,0]:=packed_data[y,k]; inc(k);
        result[y,x,2]:=packed_data[y,k]; inc(k); inc(x);
        greens       :=packed_data[y,k]; inc(k);
        result[y,x-4,1]:=greens and 17;
        result[y,x-3,1]:=(greens shr 1) and 17;
        result[y,x-2,1]:=(greens shr 2) and 17;
        result[y,x-1,1]:=(greens shr 3) and 17;
      end;
    end;
  end;

FUNCTION T_rgbPicture.ensureBitmap(CONST resized:boolean=false):boolean;
  FUNCTION getPixel(kx,ky:longint):T_rgbColor; inline;
    begin
      if kx<0 then kx:=0 else if kx>=SYS_WIDTH  then kx:=SYS_WIDTH-1;
      if ky<0 then ky:=0 else if ky>=SYS_HEIGHT then ky:=SYS_HEIGHT-1;
      result:=Pixels[ky,kx];
    end;
  VAR stencil:array[-1..2,-1..2,0..1] of word;
      st_low,st_high:array[0..1] of word;
  PROCEDURE initStencil(CONST iy:longint);
    VAR dx,dy:longint;
    begin
      for dy:=-1 to 2 do for dx:=-1 to 2 do begin
        halfTransform1(getPixel(dx-1,dy+iy),stencil[dx,dy,0],stencil[dx,dy,1]);
      end;
    end;

  PROCEDURE shiftStencil(CONST ix,iy:longint);
    CONST EFF_MAX=1;
    VAR dx,dy:longint;
    begin
      st_low[0]:=65535;
      st_low[1]:=65535;
      st_high[0]:=0;
      st_high[1]:=0;
      for dy:=-1 to EFF_MAX do begin
        for dx:=-1 to 1 do begin
          stencil[dx,dy]:=stencil[dx+1,dy];
          if stencil[dx,dy,0]<st_low [0] then st_low [0]:=stencil[dx,dy,0];
          if stencil[dx,dy,1]<st_low [1] then st_low [1]:=stencil[dx,dy,1];
          if stencil[dx,dy,0]>st_high[0] then st_high[0]:=stencil[dx,dy,0];
          if stencil[dx,dy,1]>st_high[1] then st_high[1]:=stencil[dx,dy,1];
        end;
        halfTransform1(getPixel(ix+EFF_MAX,dy+iy),stencil[EFF_MAX,dy,0],stencil[EFF_MAX,dy,1]);
        if stencil[EFF_MAX,dy,0]<st_low [0] then st_low [0]:=stencil[EFF_MAX,dy,0];
        if stencil[EFF_MAX,dy,1]<st_low [1] then st_low [1]:=stencil[EFF_MAX,dy,1];
        if stencil[EFF_MAX,dy,0]>st_high[0] then st_high[0]:=stencil[EFF_MAX,dy,0];
        if stencil[EFF_MAX,dy,1]>st_high[1] then st_high[1]:=stencil[EFF_MAX,dy,1];
      end;
    end;

  FUNCTION div256(CONST x:longint):longint; inline;
    begin
      if x<0 then result:=0 else result:=x shr 8;
    end;

  VAR line: array [0..2] of PBGRAPixel;
  PROCEDURE reconstruct_block_hq(CONST x0:longint);
    VAR k:longint;
        trueRb:array[-1..1,-1..1,0..1] of longint;
        dy,dx,j:longint;
        switch:byte;
        threshold:longint=0;
    begin
      for k:=0 to 1 do begin
        threshold:=((sqr(stencil[-1,-1,k])+sqr(stencil[-1,0,k])+sqr(stencil[-1,1,k])
                    +sqr(stencil[ 0,-1,k])+sqr(stencil[ 0,0,k])+sqr(stencil[ 0,1,k])
                    +sqr(stencil[ 1,-1,k])+sqr(stencil[ 1,0,k])+sqr(stencil[ 1,1,k])) div 9
                  - sqr((stencil[-1,-1,k] +    stencil[-1,0,k] +    stencil[-1,1,k]
                        +stencil[ 0,-1,k] +    stencil[ 0,0,k] +    stencil[ 0,1,k]
                        +stencil[ 1,-1,k] +    stencil[ 1,0,k] +    stencil[ 1,1,k] ) div 9))*2;
        j:=1; switch:=0;
        for dx:=-1 to 1 do for dy:=-1 to 1 do if (dx<>0) or (dy<>0) then begin
          if sqr(stencil[0,0,k]-stencil[dx,dy,k])<=threshold then switch+=j;
          j+=j;
        end;
        case switch of
        {$i hq3x_cases.inc}
        end;
      end;
      for dy:=0 to 2 do for dx:=0 to 2 do begin
        line[dy][x0+dx]:=halfTransform2(trueRb[dx-1,dy-1,0],trueRb[dx-1,dy-1,1]);
      end;
    end;

  VAR x,y:longint;
  begin
    EnterCriticalSection(cs);
    if Bitmap<>nil then begin
      LeaveCriticalSection(cs);
      exit(false);
    end;
    if resized then begin
      Bitmap:=TBGRABitmap.create(SYS_WIDTH*3,SYS_HEIGHT*3);
      for y:=0 to SYS_HEIGHT-1 do begin
        initStencil(y);
        line[0]:=Bitmap.ScanLine[3*y  ];
        line[1]:=Bitmap.ScanLine[3*y+1];
        line[2]:=Bitmap.ScanLine[3*y+2];
        for x:=0 to SYS_WIDTH-1 do begin
          shiftStencil(x,y);
          reconstruct_block_hq(3*x);
        end;
      end;
    end else begin
      Bitmap:=TBGRABitmap.create(SYS_WIDTH,SYS_HEIGHT);
      for y:=0 to SYS_HEIGHT-1 do begin
        line[0]:=Bitmap.ScanLine[y];
        for x:=0 to SYS_WIDTH-1 do line[0][x]:=transformColor(Pixels[y,x]);
      end;
    end;
    LeaveCriticalSection(cs);
    result:=true;
  end;

PROCEDURE T_rgbPicture.invalidateBitmap;
  begin
    EnterCriticalSection(cs);
    if Bitmap<>nil then FreeAndNil(Bitmap);
    LeaveCriticalSection(cs);
  end;

PROCEDURE T_rgbPicture.copyToImage(VAR destImage: TImage);
  begin
    ensureBitmap();
    destImage.picture.Bitmap.setSize(Bitmap.width,Bitmap.height);
    Bitmap.draw(destImage.Canvas,0,0);
    destImage.Invalidate;
  end;

PROCEDURE T_rgbPicture.writeToStream(stream: TFileStream; CONST picIndex: longint);
  VAR packed_data:T_packedFrameData;
  begin
    stream.Seek(PACKED_FRAME_DATA_SIZE*picIndex,soBeginning);
    packed_data:=pack(Pixels);
    stream.write(packed_data,PACKED_FRAME_DATA_SIZE);
  end;

PROCEDURE T_rgbPicture.loadFromStream(stream: TFileStream; CONST picIndex: longint);
  VAR packed_data:T_packedFrameData;
  begin
    stream.Seek(PACKED_FRAME_DATA_SIZE*picIndex,soBeginning);
    stream.read(packed_data,PACKED_FRAME_DATA_SIZE);
    Pixels:=unpack(packed_data);
    invalidateBitmap;
  end;

PROCEDURE T_rgbPicture.exportToPng(CONST fileName: string);
  VAR stream:TFileStream;
  begin
    ensureBitmap();
    stream:=TFileStream.create(fileName,fmCreate or fmOpenWrite);
    Bitmap.SaveToStreamAsPng(stream);
    stream.destroy;
  end;

FINALIZATION
  while exportThreadsRunning>0 do sleep(1);

end.

