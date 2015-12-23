w <- read.table("Leaf-KD-KNN.txt")
names(w) <- c("LeafSize", "Time")
ggplot(w, aes(x = LeafSize, y = Time)) + geom_line()

