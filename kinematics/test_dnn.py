from keras.models import load_model
import numpy as np

# Load the data
input_data = np.loadtxt('./data/testing_input.csv', delimiter=',')
output_data = np.loadtxt('./data/testing_output.csv', delimiter=',')

# Load the model
model = load_model('./models/my_model.h5')
model.compile(loss='mean_squared_error', optimizer='adam', metrics=['accuracy'])

# Evaluate the model
loss, accuracy = model.evaluate(input_data, output_data)

print('Loss: %.3f' % (loss))
print('Accuracy: %.3f' % (accuracy*100))