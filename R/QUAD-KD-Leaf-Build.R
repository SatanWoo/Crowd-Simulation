w <- read.table("QUAD-KD-Leaf-Build.txt", header = F)
names(w) <- c("Type", "Leaf", "Time")
ggplot(w, aes(x = Leaf, y = Time, colour = Type)) + geom_line()
