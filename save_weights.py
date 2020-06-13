#!/usr/bin/env python
import numpy as np
import scipy.special as sp

class NeuralNetwork:
    def __init__(self,input_layer,hidden_layer,output_layer,learning_late):
        self.input = input_layer 
        self.hidden = hidden_layer
        self.output = output_layer 
        self.eta = learning_late 
        self.w_ih = np.random.normal(0.0,pow(self.hidden,-0.5),(self.hidden,self.input))
        self.w_ho = np.random.normal(0.0,pow(self.output,-0.5),(self.output,self.hidden))

    def train(self,input_data,target_label):
        x_input = input_data
        target = target_label

        u_hidden = np.dot(np.array(self.w_ih),x_input)
        x_hidden = sp.expit(u_hidden)
        u_output = np.dot(np.array(self.w_ho),x_hidden)
        x_output = sp.expit(u_output)

        error = np.array(target - x_output)

        sigmoid_prime_output = np.array(x_output * (1-x_output))
        error_output_mat = np.mat(error * sigmoid_prime_output)
        error_output_array = error * sigmoid_prime_output

        x_hidden_mat = x_hidden[np.newaxis, :]
        self.w_ho = self.w_ho + self.eta * np.dot(error_output_mat.T,x_hidden_mat) 

        sigmoid_prime_hidden = np.array(x_hidden * (1-x_hidden))
        error_hidden = np.array(np.dot(self.w_ho.T,error_output_array))[0] * sigmoid_prime_hidden

        x_input_mat = x_input[np.newaxis, :]
        error_hidden_mat = np.mat(error_hidden)
        self.w_ih = self.w_ih + self.eta * np.dot(error_hidden_mat.T,x_input_mat)

def zscore(x):
        xmean = x.mean(keepdims=True)
        xstd = np.std(x,keepdims=True)
        zscore = (x-xmean)/xstd
        return zscore

def main():
    d0 = np.load('samples0_2.npy')
    d1 = np.load('samples1_2.npy')
    d2 = np.load('samples2_2.npy')
    d3 = np.load('samples3_2.npy')
    d4 = np.load('samples4_2.npy')
    train_data = np.r_[d0, np.r_[d1, np.r_[d2, np.r_[d3, d4]]]]
    train_data = np.random.permutation(train_data)
    print(train_data.shape)
    train_label = train_data[:,0]
    train_feature = train_data[:,1:]
    train_feature = train_feature.astype(np.float32)
    train_feature = zscore(train_feature)
    train_bias = np.ones(len(train_feature))
    train_feature = np.c_[train_bias,train_feature]
    print(train_feature)
    input_layer = 3073
    hidden_layer = 100
    output_layer = 5
    learning_late = 0.1
    n = NeuralNetwork(input_layer,hidden_layer,output_layer,learning_late)
    for j in range(10):
        for i in range(len(train_data)):
            targets = np.zeros(5)
            targets[int(train_label[i])]=1
            n.train(train_feature[i],targets)
        n.eta = 0.9 * n.eta
    print(n.w_ih.shape)
    np.save('w_ih2',n.w_ih)
    np.save('w_ho2',n.w_ho)
    print("save completed")

main()
