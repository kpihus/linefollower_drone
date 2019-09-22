import multiprocessing

def calc_square(number):
    print('Square'+str(number * number))
    result = number * number
    while True:
        print(result)

def calc_quad(number):
    while True:
        print('Quad'+ str(number * number * number * number))
if __name__ == "__main__":
    number = 7
    result = None
    p1 = multiprocessing.Process(target=calc_square, args=(number,))
    p2 = multiprocessing.Process(target=calc_quad, args=(number,))
    p1.start()
    p2.start()
    p1.join()
    p2.join()

    # Wont print because processes run using their own memory location
    print(result)