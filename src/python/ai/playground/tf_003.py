
# Python
import tensorflow as tf

if __name__ == '__main__':

    hello = tf.constant('Hello, TensorFlow!')
    sess = tf.Session()


    print(sess.run(hello))
