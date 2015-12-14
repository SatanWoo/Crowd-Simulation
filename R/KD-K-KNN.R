y <- read.table("KD-K-Knn.txt", header = F)
names(y) <- c("K", "Time")
ggplot(y, aes(x = K, y = Time)) + geom_line()

