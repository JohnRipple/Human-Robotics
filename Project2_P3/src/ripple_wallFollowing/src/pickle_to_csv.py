#!/usr/bin/env python

import pickle as pkl
import pandas as pd
with open("qTable.pkl", "rb") as f:
    object = pkl.load(f)
    
df = pd.DataFrame(object)
df.to_csv(r'file.csv')