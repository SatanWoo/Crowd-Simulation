setwd("~/Documents/Crowd Simulation/Crowd Simulation")
w1 <- read.table("CrossScene-FPS-560.txt", header = F)
w2 <- read.table("CrossScene-FPS-5600.txt", header = F)
w3 <- read.table("CrossScene-FPS-11200.txt", header = F)
w4 <- read.table("CrossScene-FPS-16800.txt", header = F)

data <- matrix(c(mean(w1[1,]), mean(w2[1,]), mean(w3[1,]), mean(w4[1,]), 
                 mean(w1[2,]), mean(w2[2,]), mean(w3[2,]), mean(w4[2,])), ncol = 2)

frame <- data.frame(data)
names(frame) <- c("count", "fps")
ggplot(haha, aes(x = count, y = fps)) + geom_line()