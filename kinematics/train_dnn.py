import os
import numpy as np
from tensorflow.keras.layers import Dense
from tensorflow.keras.models import Sequential

# load the dataset
input_data = np.loadtxt('./data/training_input.csv', delimiter=',')
output_data = np.loadtxt('./data/training_output.csv', delimiter=',')

# define the keras model
model = Sequential()
model.add(Dense(12, input_shape=(8,), activation='relu'))
model.add(Dense(8, activation='relu'))
model.add(Dense(6, activation='relu'))
model.add(Dense(3))

# compile the keras model
model.compile(loss='mean_squared_error', optimizer='adam', metrics=['accuracy'])

# fit the keras model on the dataset
model.fit(input_data, output_data, epochs=150, batch_size=10)

# evaluate the keras model
loss, accuracy = model.evaluate(input_data, output_data)

print('Loss: %.3f' % (loss))
print('Accuracy: %.3f' % (accuracy*100))

# Create model dir
if not os.path.exists("./models"):
    os.makedirs("./models")  

# Save then model
model.save("./models/my_model.h5")