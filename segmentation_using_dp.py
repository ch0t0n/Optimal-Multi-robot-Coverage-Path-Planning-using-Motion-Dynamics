class Segmentation: # For diving a sequence into k segments using dynamic programming
    def __init__(self, seq, k):
        self.seq = seq
        self.k = k
        self.n = len(seq)
        self.M = self.avg()  # Parameter M is taken to be the average of the sequence
        self.opt_val, self.opt_segments = self.opt()  # Calculate optimal values and segmentation

    def avg(self):  # To calculate average of sequence seq
        return sum(self.seq) / self.k

    def is_odd(self, val):  # To return a boolean value 0 or 1
        return val % 2

    def initialize_cost_and_dp(self):  # To initialize cost array, DP array and traceback array
        cost_sum = [0] * (self.n + 1)  # Initialize cost array with zeros
        dp = [[-1.0 for _ in range(self.n+1)] for _ in range(2)]  # Tabulation array for DP
        tb = [[[] for _ in range(self.n+1)] for _ in range(2)]  # Traceback array for optimal segmentation
        for i in range(0, self.n):  # O(n)
            # Add the elements of the sequence seq into cost array
            cost_sum[i + 1] = self.seq[i] + cost_sum[i]
            # Initialize DP array with infinity values
            dp[0][i+1] = float('inf')
            dp[1][i+1] = float('inf')
        # print(cost_sum, dp, tb)
        return cost_sum, dp, tb

    def generate_segmentation(self, traceback):  # To generate k-segmentation of sequence seq using traceback array
        # Separate the traceback array into two parts starts and ends
        starts = traceback[self.is_odd(self.k)][self.n]
        ends = starts[1:] + [self.n]
        # print(starts, ends)
        k_segments = [self.seq[s:e] for s, e in zip(starts, ends)]  # Concatenate the two parts for optimal segments
        return k_segments

    def opt(self):  # To calculate the optimal value and segmentation using dp
        if self.n < self.k:
            print("Not enough cells for k-segmentation, now exiting the program...")
            raise SystemExit  # Program will terminate if k is less than the segmented cells
        costSum, dp, tb = self.initialize_cost_and_dp() # Get the initial values of cost array, DP array and TB array
        for i in range(1, self.k+1):  # O(k)
            for j in range(1, self.n+1):  # O(n)
                for l in range(i-1, j):  # O(n-k)
                    curr_val = max(dp[self.is_odd(i-1)][l],
                                   abs((costSum[j] - costSum[l]) - self.M))  # Check the current max value
                    prev_val = dp[self.is_odd(i)][j]  # Check the previous value
                    if curr_val < prev_val:  # Check the min value and update it in DP and traceback array
                        dp[self.is_odd(i)][j] = curr_val  # Update the min value in dp array
                        tb[self.is_odd(i)][j] = tb[self.is_odd(i-1)][l] + [l]  # Update the breakpoint in traceback array

        optimal_value = dp[self.is_odd(self.k)][self.n]  # Calculate the optimal value
        optimal_segments = self.generate_segmentation(tb)  # Generate the optimal segments
        return optimal_value, optimal_segments