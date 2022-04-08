#include <iostream>
#include <fstream>
#define nmax 1100


TF1* flin;
double Baseline(TGraph* gWave, double & p1, int fmax=1024){
  gWave->Fit("flin","Q","",0,fmax);
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
  sleep(3);
  //delete gWave;
  delete cplot;
  return;
}

void RootWave(TString fname, TString fdata, int gverbose=0){
  // fdata è l'output di stat --printf='%.Y' fname  
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
  
  TString fout = fname;
  fout.ReplaceAll(".txt",".root");
  TFile ffout(fout.Data(), "recreate");
  TTree tree("tree", "tree");
  int header[2];
  tree.Branch("header",header,"event/I:ncell/I");

  double reco[11];
  tree.Branch("reco",reco,"vmin/D:vmax/D:vsmin/D:vsmax/D:Energy/D:Time/D:STime/D:tsmin/D:tsmax/D:p0/D:p1/D");
  int timi[2];
  tree.Branch("timi",timi,"tmin/I:tmax/I");
  Long64_t  tstamp;
  tree.Branch("tstamp",&tstamp,"tstamp/L");
  double wave[nmax];
  
  ifstream myfile;
  myfile.open(Form("%s",fname.Data()),ifstream::in);
  if (gverbose>0) cout << " open " << fname.Data() << endl;

  
  TString string1;
  TString string2;
  TString string3;
  TString string4;
  int npt=0;
  int nch=0;
  int ndum=0;
  int istart=1;
  while (!myfile.eof()){
    myfile >> string1 >> string2 >> npt;
    if(!(string1.Contains("Record"))) break;
    if(istart==1) {istart=0; tree.Branch("wave",wave,Form("wave[%d]/D",npt));}
      
    if (gverbose>0) cout << string1.Data() << " "  << npt << endl;
    myfile >> string1 >> ndum; // board number
    myfile >> string1 >> nch; // channel number
    if (gverbose>0) cout << string1.Data() << " "  << nch << endl;
    myfile >> string1 >> string2 >> header[0]; // event number
    if (gverbose>0) cout << string1.Data() << " "  << header[0] << endl;
    myfile >> string1 >> string2; //pattern
    myfile >> string1 >> string2 >> string3 >> tstamp; //tstamp
    if (gverbose>0) cout << string2.Data() << " "  << tstamp << endl;
    myfile >> string1 >> string2 >> string3 >> string4; //dc offset
    myfile >> string1 >> string2 >> string3 >> header[1]; //ncell
    if (gverbose>0) cout << string2.Data() << " "  << header[1] << endl;



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

    //******** BUTTO VIA IL PRIMO SAMPLE PERCHE' SO CHE FA SCHIFO IN QUESTO DIGITIZER
    double butto=0;
    myfile >> butto;
    npt--;
    //******** BUTTO VIA IL PRIMO SAMPLE PERCHE' SO CHE FA SCHIFO IN QUESTO DIGITIZER

    
    for(int ipt = 0; ipt<200; ipt++) {smwave[ipt]=0;}
    for(int ipt = 0; ipt<npt; ipt++) { 
      myfile >> wave[ipt]; smwave[ipt/10]+=wave[ipt]/10.; //crea una smoothwave mediando 10pt
      if(wave[ipt]<vmin) {vmin=wave[ipt]; tmin=ipt;}      
      if(wave[ipt]>vmax) {vmax=wave[ipt]; tmax=ipt;}
    }
    for(int ipt = 0; ipt<100; ipt++) {
      ewave[ipt]=(smwave[ipt]-(smwave[0]+smwave[99]+smwave[98]+smwave[97])/4.);
      ewave[ipt]=0.1*ewave[ipt]*ewave[ipt]+20.;
      //definisco un errore per fare baseline ... errore grande dove c'e' segnale
      if(smwave[ipt]<vsmin) {vsmin=smwave[ipt]; tsmin=seqx[ipt]; ismin=ipt;}
      if(smwave[ipt]>vsmax) {vsmax=smwave[ipt]; tsmax=seqx[ipt]; ismax=ipt;}
    }
    TGraph* gWave0 = new TGraph(npt,seqnum,wave);
    TGraphErrors* gWave = new TGraphErrors(100,seqx,smwave,ex,ewave);
    //    gWave->Print("all");
    double p1=0;
    double p0 = Baseline(gWave,p1,800);
    double Energy=0;
    for(int ipt = 0; ipt<100; ipt++) {Energy+=-(smwave[ipt]-(seqx[ipt]*p1+p0));}

    double thr=0.5*(vsmin+tsmin*p1+p0);
    int ithr=0;
    for(int ipt = ismin; ipt>0; ipt--) if(smwave[ipt]>thr){ithr=ipt; break;} //find the position
    double t1 = seqx[ithr-1];
    double t2 = seqx[ithr+1];
    if(ithr-1<0) {t1 = seqx[0];t2 = seqx[1];}
    double y1 = gWave0->Eval(t1);
    double y2 = gWave0->Eval(t2);
    double t0 = t1 + (thr-y1)*(t2-t1)/(y2-y1); //TGrapg extrapolation
    double sy1 = gWave->Eval(t1);
    double sy2 = gWave->Eval(t2);
    double s0 = t1 + (thr-sy1)*(t2-t1)/(sy2-sy1); //TGrapg extrapolation
    if (gverbose>6) cout << t1 << " " << t2 << " " << thr <<  " " << y1 << " " << y2 << " " << sy1 << " " << sy2 << " " << t0 << " " << s0 << " " << ismin << endl;
    
    //    if(t0-s0>5 && Energy>100)       PlotWave(gWave,gWave0);
    if (gverbose>2) {
      PlotWave(gWave,gWave0);
    }
    delete gWave;
    delete gWave0;

    reco[0]=vmin; //voltaggio minimo
    reco[1]=vmax; //voltaggio massimo
    reco[2]=vsmin; // voltaggio mediato minimo
    reco[3]=vsmax; // voltaggio mediato massimo
    reco[4]=Energy; // area picco - baseline
    reco[5]=t0; // tempo di attraversamento della soglia a 50% del picco negativo smoothato
    reco[6]=s0; // tempo smoothato di attraversamento della soglia a 50% del picco negativo smoothato
    reco[7]=tsmin; //#sample del voltaggio mediato minimo
    reco[8]=tsmax; //#sample del voltaggio mediato massimo
    reco[9]=p0; // parametri della baseline
    reco[10]=p1; //parametri dela baseline

    timi[0]=tmin; //#sample del voltaggio minimo
    timi[1]=tmax; //#sample del voltaggio massimo

    tree.Fill();
    if (gverbose == 7) break;
  }
  TNamed* datafile = new TNamed("UnixTimeLastEvent",fdata.Data());
  tree.Write();
  datafile->Write();
  myfile.close();
  ffout.Close();
  return;
}
