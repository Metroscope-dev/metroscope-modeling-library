within MetroscopeModelingLibrary.Tests.SteelworksGases.Pipes;
model Pipe_faulty "Steady operating point with 20% fouling"
  extends Pipe_direct(pipe(faulty=true));
equation
  pipe.fouling = 20;
end Pipe_faulty;
