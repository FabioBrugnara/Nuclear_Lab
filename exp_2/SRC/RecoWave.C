#include <iostream>
#include <fstream>
#define nmax 1100


int FindThr(int iud,int isugg,double thr,double wave[]){
  // negative pulses
  // sanity check
  if(iud>0) {while(wave[isugg]>thr) isugg++;}  // check in this scase isugg should be already below threshold
  if(iud<0) {while(wave[isugg]<thr) isugg--;}  // check in this scase isugg should be already above threshold
  int i = isugg;
  if(iud>0) {while(wave[i]<thr) i--;}  // i contains the last sample above threshold
  if(iud<0) {while(wave[i]>thr) i++;}  // i contains the first sample below threshold

  if(iud>0) return i+1;  // return sample first position after (last time) threshold
  if(iud<0) return i-1;  // return sample just first time above the threshold
  return isugg;
}

TF1* flin;
double Baseline(TGraph* gWave, double & p1){
  gWave->Fit("flin","Q");
  p1 = flin->GetParameter(1);
  return flin->GetParameter(0);
}


void PlotWave(TGraph* gWave,TGraph* gWave0){
  TCanvas*  cplot = new TCanvas();
  gWave0->SetMarkerStyle(8);
  gWave->SetMarkerColor(4);
  gWave->SetMarkerStyle(8);
  gWave0->Draw("ap");
  gWave->Draw("psame");
  flin->Draw("same");
  cplot->Modified();
  cplot->Update();
  cplot->SaveAs("plot.png");
  //  system("eog plot.png");
  sleep(2);
  //delete gWave;
  delete cplot;
  return;
}

void RecoWave(TString fname, int fixpend=0, int gverbose=0, int eventplot=-1){
  TH1D* hx = new TH1D("hx","",2000,-200,200);
  TH1D* nx = new TH1D("hx","",2000,-200,200);
  TH1D* hxx = new TH1D("hxx","",2000,-200,200);

  double seqnum[nmax];
  double seqx[200];
  double smwave[200];
  double ewave[200];
  double ex[200];
  for(int ipt = 0; ipt<200; ipt++) {seqx[ipt]=0;ex[ipt]=0;}
  for(int ipt = 0; ipt<nmax; ipt++) {seqnum[ipt]=ipt; seqx[ipt/10]+=seqnum[ipt]/10.;}


  flin=new TF1("flin","[1]*x+[0]",0,1000); //to find baseline
  flin->SetParameter(0,3000);
  flin->SetParameter(1,0);
  if(fixpend==1) flin->FixParameter(1,0);
    
  TFile* f = new TFile(fname.Data());
  TTree* tree = (TTree*)f->Get("tree");


  TString fout = fname;
  fout.ReplaceAll(".root","_reco.root");
  TFile ffout(fout.Data(), "recreate");
  TTree treeout("tree", "tree");
  double reco[14];
  treeout.Branch("reco",reco,"vmin/D:vmax/D:vsmin/D:vsmax/D:Energy/D:Time/D:STime/D:tsmin/D:tsmax/D:p0/D:p1/D:AEnergy/D:Pend/D:SPend/D");
  int ireco[8];
  treeout.Branch("ireco",ireco,"imin/I:imax/I:nat600/I:nat300/I:nat120/I:natm2500/I:natm1250/I:natm500/I");
  int header[2];
  treeout.Branch("header",header,"event/I:ncell/I");
  Long64_t  tstamp;
  treeout.Branch("tstamp",&tstamp,"tstamp/L");


  
  double wave[nmax];
  tree->SetBranchAddress("wave",wave);
  tree->SetBranchAddress("header",header);
  tree->SetBranchAddress("tstamp",&tstamp);

  int npt=1024;
  for (Int_t i = 0; i<tree->GetEntries(); i++) {
      tree->GetEntry(i);

      double vmin=1.e16;
      double vmax=-1;
      int tmin=0;
      int tmax=0;
      
      double vsmin=1.e16;
      double vsmax=-1;
      double tsmin=0;
      double tsmax=0;
      int ismin=0;
      int ismax=0;
      int natm2500=0;
      int natm1250=0;
      int natm500=0;
      int nat600=0;
      int nat300=0;
      int nat120=0;
      wave[0]=wave[1];
      //  TRUCCO PER BUTTARE IL PRIMO SAMPLE PERCHE' SO CHE FA SCHIFO IN QUESTO DIGITIZER (in realtà anche gli ultimi)

      double bs=0;
      double bbs=0;
      for(int ipt = 1; ipt<=20; ipt++) {bs+=wave[ipt]; bbs+=wave[ipt]*wave[ipt];}
      bs=bs/20.; // media
      bbs=sqrt(bbs/20.-bs*bs); // RMS
      
      for(int ipt = 0; ipt<200; ipt++) {smwave[ipt]=0;}
      for(int ipt = 0; ipt<npt; ipt++) { 
	smwave[ipt/10]+=wave[ipt]/10.; //crea una smoothwave mediando 10pt
	if(ipt<1000){
	  if(wave[ipt]<vmin) {vmin=wave[ipt]; tmin=ipt;}
	  if(wave[ipt]>vmax) {vmax=wave[ipt]; tmax=ipt;}      
	  if(wave[ipt]<bs-2500) {natm2500++;}
	  if(wave[ipt]<bs-1250) {natm1250++;}
	  if(wave[ipt]<bs-500) {natm500++;}
	  if(wave[ipt]>bs+600) {nat600++;}
	  if(wave[ipt]>bs+300) {nat300++;}
	  if(wave[ipt]>bs+120) {nat120++;}
	}
      }

      
      for(int ipt = 0; ipt<100; ipt++) {
	ewave[ipt]=abs(smwave[ipt]-bs)+bbs;
	if(fixpend==1 && ipt>20) ewave[ipt]=5.*ewave[ipt];
	//definisco un errore per fare baseline ... errore grande dove c'e' segnale
	if(smwave[ipt]<vsmin) {vsmin=smwave[ipt]; tsmin=seqx[ipt]; ismin=ipt;}
	if(smwave[ipt]>vsmax) {vsmax=smwave[ipt]; tsmax=seqx[ipt]; ismax=ipt;}
      }
      TGraph* gWave0 = new TGraph(npt,seqnum,wave);
      TGraphErrors* gWave = new TGraphErrors(100,seqx,smwave,ex,ewave); //di fatto butto gli ultimi punti di wave (che fan schifo)
      
      double p1=0;
      flin->SetParameter(0,bs);
      double p0 = Baseline(gWave,p1);
      double Energy=0;
      double EEnergy=0;
      for(int ipt = 0; ipt<100; ipt++) {double ddelta=-(smwave[ipt]-(seqx[ipt]*p1+p0)); Energy+=ddelta; EEnergy+=abs(ddelta);}
      
      double thr=0.5*(vsmin+tsmin*p1+p0);
      int ithr=0;
      for(int ipt = ismin; ipt>0; ipt--) if(smwave[ipt]>thr){ithr=ipt; break;} //find the position

      int iup = FindThr(1,10*(ithr+1),thr,wave);
      int idw = FindThr(-1,10*ithr,thr,wave);
      if (idw<0) idw = 0;
      double st1 = seqx[ithr];
      double st2 = seqx[ithr+1];
      double t1 = seqnum[idw];
      double t2 = seqnum[iup];
      if(ithr<0) {t1 = seqx[0];t2 = seqx[1];}
      double y1 = gWave0->Eval(t1);
      double y2 = gWave0->Eval(t2);
      double pend = (y2-y1)/(t2-t1);
      double t0 = t1 + (thr-y1)/pend; //TGrapg extrapolation
      double sy1 = gWave->Eval(st1);
      double sy2 = gWave->Eval(st2);
      double Spend = (sy2-sy1)/(st2-st1);
      double st0 = st1 + (thr-sy1)/Spend; //TGrapg extrapolation
      if (gverbose>2 || eventplot==header[0]) {
	cout << eventplot << " " << header[0] << endl;
	PlotWave(gWave,gWave0);
      }
      

      if (gverbose>6) cout << idw << " " << iup << " | " << t1 << " " << t2 << " | " << thr <<  " | " << y1 << " " << y2 << " | " << t0 << endl;
      if (gverbose>6) cout << ithr << " " << ithr+1 << " | " << st1 << " " << st2 << " | " << thr <<  " | " << sy1 << " " << sy2 << " | " << st0 << endl;
			

      if (gverbose>7) return;


      reco[0]=vmin; //voltaggio minimo
      reco[1]=vmax; //voltaggio massimo
      reco[2]=vsmin; // voltaggio mediato minimo
      reco[3]=vsmax; // voltaggio mediato massimo
      reco[4]=Energy; // area picco - baseline
      reco[5]=t0/5.; // tempo di attraversamento della soglia a 50% del picco negativo smoothato
      reco[6]=st0/5.; // tempo smoothato di attraversamento della soglia a 50% del picco negativo smoothato
      reco[7]=tsmin/5.; //#sample del voltaggio mediato minimo
      reco[8]=tsmax/5.; //#sample del voltaggio mediato massimo
      reco[9]=p0; // parametri della baseline
      reco[10]=p1; //parametri della baseline
      reco[11]=EEnergy; // absolute signal area-baseline
      reco[12]=-pend; // signal rising pendence (another proxy of the energy)
      reco[13]=-Spend; // signal rising pendence (another proxy of the energy)
      ireco[0]=tmin; //#sample del voltaggio minimo
      ireco[1]=tmax; //#sample del voltaggio massimo
      ireco[2]=nat600; //#sample 600 counts sopra baseline
      ireco[3]=nat300; 
      ireco[4]=nat120; //#sample 120 counts sopra baseline
      ireco[5]=natm2500; //#sample 2500 counts sotto baseline
      ireco[6]=natm1250;
      ireco[7]=natm500; //#sample 500 counts sotto baseline
      treeout.Fill();


      if(t0>20 && t0<200){ // only normal t0
	for(int ipt=0;ipt<npt;ipt++){
	  //	double val=(wave[ipt]-(seqnum[ipt]*p1+p0)); // questo è non normalizzato balla di più
	  // double val=(wave[ipt]-(seqnum[ipt]*p1+p0))/abs(Energy); //non so perchè questo fa barre errori enormi
	  double val=(wave[ipt]-(seqnum[ipt]*p1+p0))/abs(EEnergy); 
	  //	double val=(wave[ipt]-(seqnum[ipt]*p1+p0))/abs(vsmin-vsmax);
	  int ib = hx->FindBin((seqnum[ipt]-t0)/5.); //5Gs/s
	  nx->AddBinContent(ib,1);
	  hx->AddBinContent(ib,-val);
	  hxx->AddBinContent(ib,val*val);
	}
      }
    
      delete gWave;
      delete gWave0;


      
   }
  treeout.Write();
  ffout.Close();
  
  for(int ix = 0 ; ix<hx->GetNbinsX(); ix++){
    double xv = hx->GetBinContent(ix);
    double nn = nx->GetBinContent(ix); 
    double vvq = hxx->GetBinContent(ix);
    double vmed=0;
    double vsig=0;
    if(nn>0) {
      vmed = xv/nn;
      vsig = sqrt(vvq/nn - vmed*vmed);
    }
    hx->SetBinContent(ix,vmed);
    hx->SetBinError(ix,vsig);
  }
  hx->Draw();
  fout.ReplaceAll("_reco.root","_hist.root");
  hx->SaveAs(fout.Data());

  TF1* fgrow = new TF1("fgrow","[0]+[1]*exp(x/[2])",-10,10);
  fgrow->SetParameter(0,0.);
  fgrow->SetParameter(1,0.1);
  fgrow->SetParameter(2,0.5);
  hx->Fit("fgrow","","same",-10,-1);

  TF1* ftau = new TF1("ftau","[0]+[1]*exp(-x/[2])",-10,20);
  ftau->FixParameter(0,0.);
  ftau->SetParameter(1,0.3);
  ftau->SetParameter(2,5);
  hx->Fit("ftau","+","",10,16);
  
  return;

}
