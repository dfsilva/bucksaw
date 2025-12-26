var cacheName = 'pid-lab-v1';
var filesToCache = [
  './',
  './index.html',
  './manifest.json',
  './betaflight_icon_dark.png',
  './betaflight_icon_light.png',
  './icon-256.png',
  './icon-1024.png',
  './maskable_icon_x512.png',
];

/* Start the service worker and cache app shell */
self.addEventListener('install', function (e) {
  self.skipWaiting();
  e.waitUntil(
    caches.open(cacheName).then(function (cache) {
      return cache.addAll(filesToCache);
    })
  );
});

/* Activate and clean up old caches */
self.addEventListener('activate', function (e) {
  e.waitUntil(
    caches.keys().then(function (keyList) {
      return Promise.all(
        keyList.map(function (key) {
          if (key !== cacheName) {
            return caches.delete(key);
          }
        })
      );
    })
  );
  return self.clients.claim();
});

/* Serve cached content when offline, refresh cache when online */
self.addEventListener('fetch', function (e) {
  // Only handle GET requests
  if (e.request.method !== 'GET') return;

  // Network First, fallback to Cache strategy
  e.respondWith(
    fetch(e.request)
      .then(function (response) {
        // If valid response, clone and cache it
        if (!response || response.status !== 200 || response.type !== 'basic') {
          return response;
        }

        var responseToCache = response.clone();
        caches.open(cacheName).then(function (cache) {
            // detailed caching could go here, for now we cache everything from origin
            // that returns 200.
          cache.put(e.request, responseToCache);
        });

        return response;
      })
      .catch(function () {
        // If network fails, try cache
        return caches.match(e.request);
      })
  );
});
