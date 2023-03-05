# Use an official Node.js runtime as a parent image
FROM node:14

# Set the working directory in the container
WORKDIR /backend

# Copy package.json and package-lock.json files to the container
COPY package.json .

# Install dependencies
RUN npm install

# Copy the rest of the application code to the container
COPY . .

# # Set environment variables for MySQL
# ENV MYSQL_USER=root \
#     MYSQL_PASSWORD=root \
#     MYSQL_DATABASE=roboserver \
#     MYSQL_HOST=127.0.0.1

# Expose port 3000 for the API
EXPOSE 3000

# Start the application
CMD node index.js 
