#include "NeuralNetwork.h"

#include <math.h>

// Creates an empty network. Call setWeights() before forward().
NeuralNetwork::NeuralNetwork()
    : weights_{nullptr, nullptr, nullptr, nullptr, nullptr, nullptr} {
}

// Creates a network with externally supplied weight arrays.
NeuralNetwork::NeuralNetwork(const NeuralNetworkWeights &weights)
    : weights_(weights) {
}

// Replaces the active weight pointers without copying the weight arrays.
void NeuralNetwork::setWeights(const NeuralNetworkWeights &weights) {
  weights_ = weights;
}

// Checks that all required weight and bias arrays are available.
bool NeuralNetwork::isConfigured() const {
  return weights_.w0 != nullptr &&
         weights_.b0 != nullptr &&
         weights_.w1 != nullptr &&
         weights_.b1 != nullptr &&
         weights_.w2 != nullptr &&
         weights_.b2 != nullptr;
}

// Computes one forward pass: input -> tanh(hidden0) -> tanh(hidden1) -> action.
bool NeuralNetwork::forward(const float observation[NN_INPUT_DIM],
                            float action[NN_OUTPUT_DIM]) {
  if (observation == nullptr || action == nullptr || !isConfigured()) {
    if (action != nullptr) {
      clearOutput(action);
    }
    return false;
  }

  for (uint8_t i = 0; i < NN_HIDDEN0_DIM; ++i) {
    hidden0_[i] = activation(
        denseNeuron(weights_.w0, weights_.b0, i, NN_INPUT_DIM, observation));
  }

  for (uint8_t i = 0; i < NN_HIDDEN1_DIM; ++i) {
    hidden1_[i] = activation(
        denseNeuron(weights_.w1, weights_.b1, i, NN_HIDDEN0_DIM, hidden0_));
  }

  for (uint8_t i = 0; i < NN_OUTPUT_DIM; ++i) {
    action[i] = denseNeuron(weights_.w2, weights_.b2, i, NN_HIDDEN1_DIM,
                            hidden1_);
  }

  return true;
}

// Clamps a value to the requested range.
float NeuralNetwork::clamp(float value, float lower, float upper) {
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

// Hidden-layer activation. It matches the exported SB3 policy tanh layers.
float NeuralNetwork::activation(float value) {
  return tanhf(value);
}

// Computes one dense-layer neuron from a row-major weight matrix.
float NeuralNetwork::denseNeuron(const float *weights,
                                 const float *bias,
                                 uint8_t row,
                                 uint8_t input_dim,
                                 const float *input) {
  float sum = bias[row];
  const uint16_t row_start = (uint16_t)row * input_dim;

  for (uint8_t j = 0; j < input_dim; ++j) {
    sum += weights[row_start + j] * input[j];
  }

  return sum;
}

// Clears output to a safe zero action when inference cannot run.
void NeuralNetwork::clearOutput(float action[NN_OUTPUT_DIM]) const {
  for (uint8_t i = 0; i < NN_OUTPUT_DIM; ++i) {
    action[i] = 0.0f;
  }
}
