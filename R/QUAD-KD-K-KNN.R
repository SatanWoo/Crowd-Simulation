w <- read.table("QUAD-KD-K-Knn.txt", header = F)
names(w) <- c("Type", "K", "Time")
ggplot(w, aes(x = K, y = Time, colour = Type)) + geom_line()
