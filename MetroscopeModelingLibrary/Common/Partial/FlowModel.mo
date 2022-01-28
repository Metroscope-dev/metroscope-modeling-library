within MetroscopeModelingLibrary.Common.Partial;
model FlowModel
  extends BasicTransportModel;

equation
  DM = 0;
  DXi = zeros(Medium.nXi);
end FlowModel;
