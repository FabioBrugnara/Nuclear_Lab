void CalSHORT(TString fname="../GROUPB/SHORT/wave_0_reco.root"){
  fname="../GROUPB/FLIP/wave_0_reco.root";
  TFile* F0 = TFile::Open(fname.Data());
  TTree* ToF0 = (TTree*) F0->Get("tree");
  ToF0->SetName("ToF0");
  ToF0->SetTitle("ToF0");
  fname.ReplaceAll("0_reco.root","1_reco.root");
  TFile* F1 = TFile::Open(fname.Data());
  TTree* ToF1 = (TTree*) F1->Get("tree");
  ToF1->SetName("ToF1");
  ToF1->SetTitle("ToF1");
  fname.ReplaceAll("1_reco.root","2_reco.root");
  TFile* F2 = TFile::Open(fname.Data());
  TTree* ToF2 = (TTree*) F2->Get("tree");
  ToF2->SetName("ToF2");
  ToF2->SetTitle("ToF2");
  fname.ReplaceAll("2_reco.root","3_reco.root");
  TFile* F3 = TFile::Open(fname.Data());
  TTree* ToF3 = (TTree*) F3->Get("tree");
  ToF3->SetName("ToF3");
  ToF3->SetTitle("ToF3");
  
  ToF0->AddFriend(ToF1);
  ToF0->AddFriend(ToF2);
  ToF0->AddFriend(ToF3);

  TCanvas * c1 = new TCanvas();
  TH1D* h1 = new TH1D("h1","",2000,-100,100);
  ToF0->Draw("ToF1.Time-ToF0.Time>>h1","","colz");
  h1->Fit("gaus","L");  
  gPad->SetLogy();
  
  TCanvas * c2 = new TCanvas();
  TH1D* h2 = new TH1D("h2","",2000,-100,100);
  ToF0->Draw("ToF2.Time-ToF0.Time>>h2","","colz");
  h2->Fit("gaus","L");
  gPad->SetLogy();

  TCanvas * c3 = new TCanvas();
  TH1D* h3 = new TH1D("h3","",2000,-100,100);
  ToF0->Draw("ToF3.Time-ToF0.Time>>h3","","colz");
  h3->Fit("gaus","L");
  gPad->SetLogy();
  return;

}
