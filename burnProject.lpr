PROGRAM burnProject;

{$mode objfpc}{$H+}

USES
  {$ifdef UNIX}{$IFDEF UseCThreads}
  cthreads,
  {$endif}{$endif}
  Interfaces, // this includes the LCL widgetset
  Forms, burnMain
  { you can add units after this };

{$R *.res}

begin
  RequireDerivedFormResource:=true;
  Application.Scaled:=true;
  Application.title:='Burn';
  Application.initialize;
  Application.CreateForm(TBurnForm, BurnForm);
  Application.run;
end.

