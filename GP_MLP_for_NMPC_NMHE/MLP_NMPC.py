#!/usr/bin/env python
# coding: utf-8

import tensorflow as tf

from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Dropout, LSTM
from tensorflow.keras.optimizers import Adam,RMSprop,Adadelta,Adamax,SGD
from tensorflow.keras.callbacks import ModelCheckpoint,EarlyStopping
import numpy as np
import pandas as pd
import seaborn as sns
import sklearn
from sklearn import preprocessing
from sklearn.preprocessing import RobustScaler
from sklearn.preprocessing import StandardScaler
from sklearn.preprocessing import MinMaxScaler
from sklearn.model_selection import train_test_split
from sklearn.metrics import r2_score

from matplotlib import pyplot as plt
get_ipython().run_line_magic('matplotlib', 'inline')
get_ipython().run_line_magic('config', "InlineBackend.figure_format='retina'")


#read the data
df = pd.read_excel('NNMPC.xlsx')
# df = pd.read_csv('data.csv')
df.head()


# split data into dataframes based on variables
ax = df['ax']
delta = df['delta']
vx = df['vx']
vy = df['vy']
wz = df['wz']
ye=y = df['y_e']
thetae = df['theta_e']
d = df['delta-1']
a = df['ax-1']


# check variables correlation
np.corrcoef(ax,a)
plt.scatter(ax,a)
plt.ylabel('ax(k+1)')
plt.xlabel('ax(k)')
plt.show()


np.corrcoef(delta,d)
plt.scatter(delta,d)
plt.ylabel('delta(k+1)')
plt.xlabel('d(k)')
plt.show()




df1 = df.iloc[0:-1, 2:9]
df2 = df.iloc[0:-1, 0:2]



# scaler = RobustScaler()
scalerx = StandardScaler()
scalery = StandardScaler()


df1 = scalerx.fit_transform(df1)

df2 = scalery.fit_transform(df2)



#Compute the mean and scale with standard scaler

scalerx.mean_

scalerx.scale_


scalery.mean_

scalery.scale_





X_train, X_test, y_train, y_test = train_test_split(df1, df2, test_size=0.2)





print(X_train.shape) 





sns.displot(X_train[:,0])


sns.displot(y_train[:,1], kind="kde",bw_adjust=.25)

# build a simple sequential model
model = Sequential()
model.add(Dense(8,activation = 'sigmoid', input_shape=(7,)))
model.add(Dense(5, activation = 'sigmoid'))
model.add(Dense(2))

#Compiles model
# model.compile(SGD(learning_rate=0.001),loss='mse')
model.compile(Adam(learning_rate=0.0005),loss='mse')
# model.compile(RMSprop(learning_rate=0.0005, rho=0.75),loss='mean_absolute_error')
# model.compile(Adadelta(learning_rate=0.0015, rho=0.95),loss='mse')
# model.compile(Adamax(learning_rate=0.001),loss='mse')


#set path and name to save the neural network
path = 'nnmpc.h5'

checkpoint = ModelCheckpoint(path, monitor="val_loss", verbose=1, save_best_only=True, mode='min')
callbacks_list = [checkpoint]


#model summary
model.summary()


#Fits model
history = model.fit(X_train, y_train, epochs = 500, validation_split = 0.25,callbacks = callbacks_list,batch_size =32,shuffle = True,verbose = 1)

history_dict=history.history
#Plots model's training cost/loss and model's validation split cost/loss
loss_values = history_dict['loss']
val_loss_values=history_dict['val_loss']


# plot learning results
fig, ax = plt.subplots()
plt.plot(loss_values,'b',label='training loss')
plt.plot(val_loss_values,'r',label='validation loss')
plt.ylabel('loss')
plt.xlabel('epoch')
plt.legend(loc='upper right')
plt.show()


#fig.savefig('loss.png', format='png', dpi=1200)

# Runs model (the one with the activation function, although this doesn't really matter as they perform the same)
# with its current weights on the training and testing data
y_train_pred = model.predict(X_train)
y_test_pred = model.predict(X_test)

# Calculates and prints r2 score of training and testing data
print("The R2 score on the Train set is:\t{:0.3f}".format(r2_score(y_train, y_train_pred)))
print("The R2 score on the Test set is:\t{:0.3f}".format(r2_score(y_test, y_test_pred)))


y_test = scalery.inverse_transform(y_test)
y_test_pred = scalery.inverse_transform(y_test_pred)


y_test_pred

#plot predictions
fig, ax = plt.subplots()
plt.title('Predictions VS Ground_truth')
plt.scatter(y_test[:,0],y_test_pred[:,0])
plt.ylabel('Predictions')
plt.xlabel('True value')
# plt.legend(['Ground_truth', 'Predictions'], loc='upper left')
plt.show()
#fig.savefig('predVStruth.png', format='png', dpi=1200)



def chart_regression(pred,y,sort=True):
    #t = pd.DataFrame({'preds':preds,'y':y.flatten()})
    #if sort:
        #t.sort.values(by = ['y'], inplace = True)
    fig, ax = plt.subplots()
    a = plt.plot(y.tolist(),label='expected')
    b = plt.plot(pred.tolist(),label='prediction')
    plt.ylabel('output')
    plt.xlabel('datapoint')
    plt.legend(loc='upper right')
    plt.show()
    fig.savefig('pred_expect.png', format='png', dpi=1200)

#Plot regression results
chart_regression(y_test_pred[60:110,0],y_test[60:110,0],sort=True)





