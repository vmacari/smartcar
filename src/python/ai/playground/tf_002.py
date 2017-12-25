import tensorflow as tf

# a + b * c
# 2  + 2 * 2

if __name__ == '__main__':

    # creates nodes in a graph
    # "construction phase"
    x1 = tf.constant(2)
    x2 = tf.constant(2)
    x3 = tf.constant(2)

    # result = tf.multiply(x1, x2)
    # result = tf.add(result, x3)


    # defines our session and launches graph
    sess = tf.Session()

    # runs result
    print(sess.run(x1 + x2 * x3))
    sess.close()

    with tf.Session() as sess:
        with tf.device("/cpu:0"):
            matrix1 = tf.constant([[3., 3.]])
            matrix2 = tf.constant([[2.],[2.]])

            product = tf.matmul(matrix1, matrix2)
            print(product)

        print(sess.run(product))


