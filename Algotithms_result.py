import pandas as pd
from Simulate.Plot import*
data=[]
legend=["GA","PSO","COA","DMOA","SCSO","HBA"]

data.append(pd.read_csv("result/evaluation_n_GA.csv").loc[0:10000])
data.append(pd.read_csv("result/evaluation_n_PSO.csv").loc[0:10000])
data.append(pd.read_csv("result/evaluation_n_COA.csv").loc[0:10000])
data.append(pd.read_csv("result/evaluation_n_DMOA.csv").loc[0:10000])
data.append(pd.read_csv("result/evaluation_n_SCSO.csv").loc[0:10000])
data.append(pd.read_csv("result/evaluation_n_HBA.csv").loc[0:10000])
plot_eval(data,legend,"Constant Speed (CoP Controller)")
print("GA:",data[0]['Fitness'].loc[10000])
print("PSO:",data[1]['Fitness'].loc[10000])
print("COA:",data[2]['Fitness'].loc[10000])
print("DMOA:",data[3]['Fitness'].loc[10000])
print("SCSO:",data[4]['Fitness'].loc[10000])
print("HBA:",data[5]['Fitness'].loc[10000])

data=[]
data.append(pd.read_csv("result/evaluation_fs_GA.csv").loc[0:10000])
data.append(pd.read_csv("result/evaluation_fs_PSO.csv").loc[0:10000])
data.append(pd.read_csv("result/evaluation_fs_COA.csv").loc[0:10000])
data.append(pd.read_csv("result/evaluation_fs_DMOA.csv").loc[0:10000])
data.append(pd.read_csv("result/evaluation_fs_SCSO.csv").loc[0:10000])
data.append(pd.read_csv("result/evaluation_fs_HBA.csv").loc[0:10000])
plot_eval(data,legend,"Fast to Slow (Hierarchical Controller)")
print("GA:",data[0]['Fitness'].loc[10000])
print("PSO:",data[1]['Fitness'].loc[10000])
print("COA:",data[2]['Fitness'].loc[10000])
print("DMOA:",data[3]['Fitness'].loc[10000])
print("SCSO:",data[4]['Fitness'].loc[10000])
print("HBA:",data[5]['Fitness'].loc[10000])

data=[]
data.append(pd.read_csv("result/evaluation_sf_GA.csv").loc[0:10000])
data.append(pd.read_csv("result/evaluation_sf_PSO.csv").loc[0:10000])
data.append(pd.read_csv("result/evaluation_sf_COA.csv").loc[0:10000])
data.append(pd.read_csv("result/evaluation_sf_DMOA.csv").loc[0:10000])
data.append(pd.read_csv("result/evaluation_sf_SCSO.csv").loc[0:10000])
data.append(pd.read_csv("result/evaluation_sf_HBA.csv").loc[0:10000])
plot_eval(data,legend,"Slow to Fast (Hierarchical Controller)")
print("GA:",data[0]['Fitness'].loc[10000])
print("PSO:",data[1]['Fitness'].loc[10000])
print("COA:",data[2]['Fitness'].loc[10000])
print("DMOA:",data[3]['Fitness'].loc[10000])
print("SCSO:",data[4]['Fitness'].loc[10000])
print("HBA:",data[5]['Fitness'].loc[10000])