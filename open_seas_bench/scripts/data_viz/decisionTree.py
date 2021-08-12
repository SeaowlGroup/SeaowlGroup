from sklearn import tree
import numpy as np
import rospkg
import graphviz
import rospkg
import tkinter.filedialog #somehow necessary
import tkinter as tk
from sklearn.metrics import confusion_matrix
import pandas as pd


class DTC(object):
    rospack = rospkg.RosPack()

    def __init__(self,name,gp):
            

        self.input = f'{rospack.get_path("open_seas_bench")}/input/{name}'
        self.output = f'{rospack.get_path("open_seas_bench")}/output/{name}'
        self.sits = []
        self.data = []
        self.target = []
        self.timeCut = 80.
        self.antCut = .2
        self.nSamples = 0
        self.nFeatures = 0
        self.ntarget = 8.
        self.dtc = None
        self.fnames = ["ASV Speed", "Obstacle Heading", "Obstacle Speed", "Theoretical dCPA", "Detection Distance"]
        classes = []
        self.perf = ['Fast','Slow']
        self.secu = ['Safe','NQSafe','NQDangerous','Dangerous']
        for l2 in self.perf:
            for l1 in self.secu:
                classes.append(f'{l1}/{l2}')
        self.cnames = [f'Not_{self.secu[gp]}',self.secu[gp]]
        #self.colors = ['green','yellow','orange','red','darkgreen','gold','darkorange','darkred']
            
        self.pred = []
        self.initData()
        self.initTarget(gp)
        self.initTree()

    def initData(self):
        f = open(self.input,'r')
        f.readline()
        for line in f:
            content = line.split()
            self.data.append([float(content[2]),float(content[4]),float(content[5]),float(content[6]),float(content[9])])

        self.data = np.array(self.data)
        (self.nSamples,self.nFeatures) = np.shape(self.data)

        f.close()

    def initTarget(self, gp=-1):
        secClass = np.zeros((4,2))
        f = open(self.output, 'r')
        f.readline()
        for line in f:
            content = line.split()
            if float(content[4]) > .2: # dangerous security indicator
                i=3
            elif float(content[4]) >= -0.2: # insecure
                if float(content[6]) > self.antCut: # 
                    i=2
                else:
                    i=1
            else : # safe
                i=0
            if float(content[1]) > self.timeCut: # too slow
                j=1
            else : # performant
                j=0
            #self.target.append(i + j*4)
            self.target.append(int(i ==gp))
                
            secClass[i,j] += 1
        f.close()

        self.target = np.array(self.target)
        
        print("Total number : ", len(self.target))
        print(' ')
        df = pd.DataFrame(secClass,columns=self.perf,index=self.secu)
        print(df)
        print(' ')


        
        if len(self.target) != self.nSamples:
            raise("unmatched input/output")

    def initTree(self):
        self.dtc = tree.DecisionTreeClassifier(max_depth=5)
        self.dtc.fit(self.data, self.target)
        self.pred = self.dtc.predict(self.data)
    
    def confMatrix(self):
        m = confusion_matrix(self.target,self.pred)
        print('True\Prediction')
        df = pd.DataFrame(m, columns=self.cnames, index=self.cnames)
        print(df)


    def show(self):
        #r = tree.export_text(self.dtc, feature_names=self.fnames)
        #print(r)
        dot_data = tree.export_graphviz(self.dtc, out_file=None,feature_names=self.fnames, class_names = self.cnames, filled=True) 
        graph = graphviz.Source(dot_data) 
        graph.render(directory=f'{rospack.get_path("open_seas_bench")}/scripts/data_viz/output',filename=f"dtc_{self.cnames[-1]}",format='png', view=True)

if __name__ == "__main__":
    rospack = rospkg.RosPack()

    filepath = tk.filedialog.askopenfilename(title="Load a file :",filetypes=[('txt files','.txt'),('all files','.*')],
                                             initialdir=f"{rospack.get_path('open_seas_bench')}/output/")
    if filepath:
        name = filepath.split('/')[-1]
    else:
        print("No file chosen")

    for gp in range(4):
        t = DTC(name,gp)
        t.confMatrix()
        t.show()