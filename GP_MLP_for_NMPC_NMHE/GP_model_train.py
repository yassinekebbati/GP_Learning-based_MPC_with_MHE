import time
import numpy as np
import _pickle as pickle
import matplotlib.pyplot as plt
from matplotlib import gridspec
import pandas as pd
from sklearn.preprocessing import StandardScaler
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.model_selection import train_test_split
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C, WhiteKernel
from sklearn.metrics import mean_squared_error, r2_score, explained_variance_score
from plots import plot_true_predicted_variance



#####################################################################
# load data

data = 'Ltrack_gp.xlsx'
#data = 'Otrack_gp.xlsx'
df = pd.read_excel('Ltrack_gp.xlsx')

x = df.iloc[:, 4:10]
y = df.iloc[:, 0:4]


#####################################################################
# scale the data
xscaler = StandardScaler()
yscaler = StandardScaler()
xscaler.fit(x)
yscaler.fit(y)

#####################################################################
# Split data into training and testing sets
xscaler.transform(x)
yscaler.transform(y)
x_train, x_test, y_train, y_test = train_test_split(x, y, test_size=0.15)

print('mean values for input data =', xscaler.mean_)
print('standard deviation values for input data =', xscaler.scale_)
print('mean values for output data =', yscaler.mean_)
print('standard deviation values for output data =', yscaler.scale_)
#####################################################################
# Build GP model

k1 = 0.1 * RBF(length_scale=np.ones(x_train.shape[1]),length_scale_bounds=(1e-2, 1e2)) 

k2 = C(constant_value=0.75, constant_value_bounds=(1e-2, 1e2))

k3 = WhiteKernel(noise_level=0.1)  # noise terms

k5 = RQ ( alpha= 1e-2, length_scale_bounds=(1e-2, 1e2))

kernel = k1 * k2 * k5 + k3

gp = GaussianProcessRegressor(alpha=1e-3, kernel=kernel, normalize_y=True,	n_restarts_optimizer=5,)


#####################################################################
# Train the GP model
start = time.time()
gp.fit(x_train, y_train)
end = time.time()

print('training time: %ss' %(end - start)) 
print('final kernel: %s' %(gp.kernel_))


#####################################################################
# test GP model on training data

y_train_pred, y_train_std = gp.predict(x_train, return_std=True)


#######################################################################
# Evaluate model accuracy on training data

y_train = yscaler.inverse_transform(y_train)
y_train_pred = yscaler.inverse_transform(y_train_pred)
y_train_std *= yscaler.scale_

MSE = mean_squared_error(y_train, y_train_pred, multioutput='raw_values')
R2Score = r2_score(y_train, y_train_pred, multioutput='raw_values')
EV = explained_variance_score(y_train, y_train_pred, multioutput='raw_values')

#####################################################################
# show GP prediction scores on training data

print('root mean square error: %s' %(np.sqrt(MSE)))
print('normalized mean square error: %s' %(np.sqrt(MSE)/np.array(np.abs(y_train.mean()))))
print('R2 score: %s' %(R2Score))
print('explained variance: %s' %(EV))

#####################################################################
# save GP model

modelfile  = "GP_model_Ltrack"

joblib.dump(gp, modelfile)

df0 = pd.DataFrame([[xscaler.mean_.T , xscaler.scale_.T ,  yscaler.mean_.T , yscaler.scale_.T]],
               index=['Values'],
               columns=['xmean', 'xstdev', 'ymean', 'ystdev'])
df0.to_excel("GP_model_Ltrack_Scale.xlsx") 


#####################################################################
# test GP model on training data

y_test_pred, y_test_std = gp.predict(x_test, return_std=True)


#####################################################################
# Evaluate model accuracy on testing

y_test = yscaler.inverse_transform(y_test)
y_test_pred = yscaler.inverse_transform(y_test_pred)
y_test_std *= yscaler.scale_

MSE = mean_squared_error(y_test, y_test_pred, multioutput='raw_values')
R2Score = r2_score(y_test, y_test_pred, multioutput='raw_values')
EV = explained_variance_score(y_test, y_test_pred, multioutput='raw_values')


#####################################################################
# show GP prediction scores on testing data

print('root mean square error: %s' %(np.sqrt(MSE)))
print('normalized mean square error: %s' %(np.sqrt(MSE)/np.array(np.abs(y_test.mean()))))
print('R2 score: %s' %(R2Score))
print('explained variance: %s' %(EV))



#####################################################################
# Plot GP prediction vs truth + 95% interval

i = range(y_test.shape[0])

# PLot prediction erro for E_x
plt.figure(figsize=(9,3))
plt.plot(i, y_test_pred[:,0], '#990000', ls='-', lw=1.5, zorder=9, label='predicted')
plt.fill_between(i, (y_test_pred[:,0]+2*y_test_std[:,0]), (y_test_pred[:,0]-2*y_test_std[:,0]), alpha=0.2, color='m', label='+-2sigma')
plt.plot(i, y_test[:,0],  '#e68a00', ls='--', lw=1, zorder=9, label='true')
plt.legend(loc='upper right')
plt.title('True vs Predicted')
#plt.xlabel('samples')
plt.ylabel('E_x Error(m)')
plt.grid()
plt.show() 


# PLot prediction erro for E_y
plt.figure(figsize=(9,3))
plt.plot(i, y_test_pred[:,1], '#990000', ls='-', lw=1.5, zorder=9, label='predicted')
plt.fill_between(i, (y_test_pred[:,1]+2*y_test_std[:,1]), (y_test_pred[:,1]-2*y_test_std[:,1]), alpha=0.2, color='m', label='+-2sigma')
plt.plot(i, y_test[:,1],  '#e68a00', ls='--', lw=1, zorder=9, label='true')
#plt.legend(loc='upper right')
#plt.title('true vs predicted')
#plt.xlabel('samples')
plt.ylabel('E_y Error(m)')
plt.grid()
plt.show() 


# PLot prediction erro for E_psi
plt.figure(figsize=(9,3))
plt.plot(i, y_test_pred[:,2], '#990000', ls='-', lw=1.5, zorder=9, label='predicted')
plt.fill_between(i, (y_test_pred[:,2]+2*y_test_std[:,2]), (y_test_pred[:,2]-2*y_test_std[:,2]), alpha=0.2, color='m', label='+-2sigma')
plt.plot(i, y_test[:,2],  '#e68a00', ls='--', lw=1, zorder=9, label='true')
#plt.legend(loc='upper right')
#plt.title('true vs predicted')
#plt.xlabel('samples')
plt.ylabel('E_psi Error(rad)')
plt.grid()
plt.show()


# PLot prediction erro for E_vx
plt.figure(figsize=(9,3))
plt.plot(i, y_test_pred[:,3], '#990000', ls='-', lw=1.5, zorder=9, label='predicted')
plt.fill_between(i, (y_test_pred[:,3]+2*y_test_std[:,3]), (y_test_pred[:,3]-2*y_test_std[:,3]), alpha=0.2, color='m', label='+-2sigma')
plt.plot(i,y_test[:,3],  '#e68a00', ls='--', lw=1, zorder=9, label='true')
#plt.legend(loc='upper right')
#plt.title('true vs predicted')
plt.xlabel('samples')
plt.ylabel('E_vx Error(m/s)')
plt.grid()
plt.show()