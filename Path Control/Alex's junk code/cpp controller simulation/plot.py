import pandas as pd
import matplotlib.pyplot as plt
df = pd.read_csv("data.csv")
plt.plot(df["x"], df["y"])
plt.show()