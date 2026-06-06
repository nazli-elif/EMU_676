/*
 * UAV Pipeline Inspection
 * Facility Location + VRP + Energy Constraints
 * CPLEX OPL Model File (.mod)  -- v2
 *
 * Degisiklikler (v1 -> v2):
 *   1. Tam sarj modeli: R parametresi kaldirildi.
 *      Dock ziyaretinde batarya her zaman Q'ya (tam kapasiteye) yukselir.
 *
 *   2. Node splitting: Her fiziksel dock j, maxVisits adet kopya dugum
 *      olarak modellendi. Boylece MTZ kisiti ayni fiziksel dock'a birden
 *      fazla ziyareti engellemez; her kopya bagimsiz bir dugum sayilir.
 *
 *   3. Kisit (7b) guncellendi -- ust sinir (tam sarj):
 *        e[j][k] <= Q * z[dockPhysical[j]][k] + BigM * (1 - x[i][j][k])
 *
 *   4. Kisit (7b') eklendi -- alt sinir:
 *        e[j][k] >= e[i][k] - dist[i][j]*x[i][j][k] - BigM*(1-x[i][j][k])
 *      (7b) ve (7b') birlikte, arc kullanilip z=1 iken e[j][k]=Q'ya kilitler.
 *
 *   5. Kisit (9) ctDockOpen: her kopya dugum icin fiziksel dock acilma
 *      karari y[j] zorlaniyor.
 *
 *   6. Kisit (10) ctZlink, (12) ctRechargeExact: tum kopya dugumlerinin
 *      arc toplamina gore guncellendi.
 *
 *   7. MTZ kisitlari nNodes_exp ile guncellendi.
 *
 * Node indexing -- genisletilmis kume V' (nP=4, nD=3, maxVisits=2 ornegi):
 *   0          = Depot
 *   1..4       = Pipeline P1..P4
 *   5, 6       = Dock A (fiziksel j=1) kopyalari
 *   7, 8       = Dock B (fiziksel j=2) kopyalari
 *   9, 10      = Dock C (fiziksel j=3) kopyalari
 *   --> nNodes_exp = 11
 *
 *   Genel formul: fiziksel dock j'nin c. kopyasi (c=0..maxVisits-1):
 *     nP + (j-1)*maxVisits + c + 1
 */


int nNodes_exp = ...;   // |V'| = 1 + nP + nD*maxVisits
int nUAV       = ...;
int nP         = ...;
int nD         = ...;
int maxVisits  = ...;   // Her fiziksel dock icin max ziyaret sayisi

range Nodes_exp = 0..nNodes_exp-1;
range UAVs      = 0..nUAV-1;
range Pipelines = 1..nP;
range DocksExp  = nP+1..nP+nD*maxVisits;
range DocksPhys = 1..nD;

float dist[Nodes_exp][Nodes_exp] = ...;
float travelCost                 = ...;
float Q                          = ...;
float fixedCost[DocksPhys]       = ...;
float inspectCost[Nodes_exp]     = ...;
int   dockPhysical[DocksExp]     = ...;   // Kopya -> fiziksel dock indeksi (1..nD)

int BigM = 10000;

dvar boolean x[Nodes_exp][Nodes_exp][UAVs];
dvar boolean y[DocksPhys];
dvar boolean z[DocksPhys][UAVs];
dvar float+  e[Nodes_exp][UAVs];
dvar float+  u[Nodes_exp][UAVs];

minimize
  sum(i in Nodes_exp, j in Nodes_exp, k in UAVs)
    travelCost * dist[i][j] * x[i][j][k]
  +
  sum(j in DocksPhys)
    fixedCost[j] * y[j];

subject to {

  // (1) Her pipeline dugumu tam olarak bir kez ziyaret edilir
  forall(i in Pipelines)
    ctVisitOnce:
      sum(j in Nodes_exp, k in UAVs) x[i][j][k] == 1;

  // (2) Her UAV icin akis dengesi
  forall(i in Nodes_exp, k in UAVs)
    ctFlowBalance:
      sum(j in Nodes_exp) x[i][j][k] == sum(j in Nodes_exp) x[j][i][k];

  // (3) Her UAV depodan tam olarak bir kez cikiyor
  forall(k in UAVs)
    ctDepotDepart:
      sum(j in Nodes_exp) x[0][j][k] == 1;

  // (4) Her UAV depoya tam olarak bir kez donuyor
  forall(k in UAVs)
    ctDepotReturn:
      sum(i in Nodes_exp) x[i][0][k] == 1;

  // (5) Oz-dongu yok
  forall(i in Nodes_exp, k in UAVs)
    ctNoSelf:
      x[i][i][k] == 0;

  // (6) Her UAV depoda tam batarya ile baslar
  forall(k in UAVs)
    ctInitBattery:
      e[0][k] == Q;

  // (7a) Pipeline dugumlerinde batarya tuketimi (seyahat + inceleme)
  forall(i in Nodes_exp, j in Pipelines, k in UAVs)
    ctBatteryDrain:
      e[j][k] <= e[i][k]
                 - dist[i][j]     * x[i][j][k]
                 - inspectCost[j] * x[i][j][k]
                 + BigM * (1 - x[i][j][k]);

  // (7b) Dock kopya dugumunde tam sarj -- ust sinir
  //      z=1 ve arc kullanilirsa: e[j][k] <= Q
  //      Arc kullanilmazsa BigM ile gevsetilir
  forall(i in Nodes_exp, j in DocksExp, k in UAVs)
    ctBatteryChargeUB:
      e[j][k] <= Q * z[dockPhysical[j]][k]
                 + BigM * (1 - x[i][j][k]);

  // (7b') Dock kopya dugumunde batarya -- alt sinir
  //       Arc kullanildiginda e[j][k], varis oncesi degerden asagi dusamaz.
  //       (7b) ile birlikte z=1 iken e[j][k] = Q'ya kilitlenir.
  forall(i in Nodes_exp, j in DocksExp, k in UAVs)
    ctBatteryChargeLB:
      e[j][k] >= e[i][k]
                 - dist[i][j] * x[i][j][k]
                 - BigM * (1 - x[i][j][k]);

  // (7c) Depoya donus icin yeterli batarya
  forall(i in Nodes_exp : i != 0, k in UAVs)
    ctBatteryReturn:
      e[i][k] >= dist[i][0] * x[i][0][k];

  // (8) Batarya kapasitesi ust siniri
  forall(i in Nodes_exp, k in UAVs)
    ctBatteryCap:
      e[i][k] <= Q;

  // (9) Herhangi bir kopya ziyaret edilirse fiziksel dock acilmali
  forall(j in DocksPhys, c in 0..maxVisits-1)
    ctDockOpen:
      sum(i in Nodes_exp, k in UAVs)
        x[i][nP + (j-1)*maxVisits + c + 1][k]
      <= BigM * y[j];

  // (10) z[j][k]: UAV k, fiziksel dock j'nin herhangi bir kopyasini ziyaret ettiyse 1
  forall(j in DocksPhys, k in UAVs)
    ctZlink:
      z[j][k] <= sum(c in 0..maxVisits-1, i in Nodes_exp)
                   x[i][nP + (j-1)*maxVisits + c + 1][k];

  // (11) Sarj yalnizca acik dock'ta yapilabilir
  forall(j in DocksPhys, k in UAVs)
    ctZopen:
      z[j][k] <= y[j];

  // (12) Dock ziyareti gerceklesirse sarj zorunlu
  //      (1/maxVisits): herhangi 1 kopya ziyareti bile z=1'i zorlar
  forall(j in DocksPhys, k in UAVs)
    ctRechargeExact:
      z[j][k] >= (1.0 / maxVisits) *
                 sum(c in 0..maxVisits-1, i in Nodes_exp)
                   x[i][nP + (j-1)*maxVisits + c + 1][k];

  // (13) MTZ alt-tur eliminasyonu -- genisletilmis V' uzerinde
  //      Kopya dock dugumleri farkli indekse sahip oldugu icin,
  //      ayni fiziksel dock'a birden fazla ziyaret artik engellenmez.
  forall(i in Nodes_exp, j in Nodes_exp : i != 0 && j != 0 && i != j, k in UAVs)
    ctMTZ:
      u[i][k] - u[j][k] + nNodes_exp * x[i][j][k] <= nNodes_exp - 1;

  // (14) MTZ sinir degerleri
  forall(i in Nodes_exp : i != 0, k in UAVs) {
    ctMTZLB: u[i][k] >= 1;
    ctMTZUB: u[i][k] <= nNodes_exp - 1;
  }

  // (15) UAV kullaniliyorsa en az bir pipeline dugumu ziyaret etmeli
  //      Depodan cikis yapan her UAV, en az bir P dugumune gitmek zorundadir.
  //      Boylece "0 -> Dock -> 0" gibi bos rotalar engellenir.
  forall(k in UAVs)
    ctMinOnePipeline:
      sum(i in Nodes_exp, j in Pipelines) x[i][j][k]
        >= sum(j in Nodes_exp) x[0][j][k];

}


execute INIT {
  cplex.tilim = 900;
}

execute DISPLAY {
 writeln("=== SOLUTION ===");
  writeln("Objective (best incumbent): ", cplex.getObjValue());
  writeln("Best bound                : ", cplex.getBestObjValue());
  writeln("Optimality gap %          : ",
    (cplex.getObjValue() - cplex.getBestObjValue())
    / cplex.getObjValue() * 100);

  writeln("Physical docking stations opened:");
  for(var j in DocksPhys) {
    if(y[j] == 1)
      writeln("  Physical dock ", j,
              "  (fixed cost: ", fixedCost[j], ")");
  }

  writeln("");
  writeln("UAV routes:");
  for(var k in UAVs) {
    write("  UAV ", k, ": 0");
    var cur = 0;
    var steps = 0;
    var done = false;
    while(!done && steps < nNodes_exp + 1) {
      for(var j in Nodes_exp) {
        if(x[cur][j][k] == 1 && j != cur) {
          if(j >= nP + 1) {
            write(" -> D", dockPhysical[j],
                  "(v", j - nP, ")",
                  "[bat=", e[j][k], "]");
          } else {
            write(" -> P", j, "[bat=", e[j][k], "]");
          }
          cur = j;
          if(j == 0) done = true;
          break;
        }
      }
      steps++;
    }
    writeln(" -> 0");
  }

  writeln("");
  writeln("Recharge events:");
  for(var k in UAVs) {
    for(var j in DocksPhys) {
      if(z[j][k] == 1)
        writeln("  UAV ", k, " recharged at physical dock ", j);
    }
  }
}