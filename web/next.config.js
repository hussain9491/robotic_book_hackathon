/** @type {import('next').NextConfig} */
const nextConfig = {
  output: 'export', // Export as a static site
  trailingSlash: true, // Add trailing slashes to URLs
  images: {
    unoptimized: true // Required for export
  },
  webpack: (config, { isServer }) => {
    if (!isServer) {
      config.resolve.fallback = {
        fs: false,
      }
    }
    return config
  }
}

module.exports = nextConfig