import os
import tqdm
l = os.listdir()

for i in tqdm.tqdm(range(len(l))):
	os.rename(l[i], str(i) + '.jpg')

