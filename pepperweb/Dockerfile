FROM node:14 as nodejs-builder

ARG API_BASE_URL
RUN if [ -z $(echo $API_BASE_URL | sed -nre "/.*\/api$/p") ]; then echo "ERROR: API_BASE_URL must end with \"/api\""; exit 1; fi

RUN apt-get update && \
    apt-get install -y \
        build-essential

COPY package.json /src/
WORKDIR /src
RUN npm install -g npm
RUN npm install

COPY . /src
RUN npm run build


FROM nginx:alpine
COPY default.conf /etc/nginx/conf.d/default.conf
COPY --from=nodejs-builder /src/dist/ /usr/share/nginx/html/
EXPOSE 8080