/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    "./src/**/*.{js,jsx,ts,tsx}",
    "./docs/**/*.{md,mdx}",
    "./blog/**/*.{md,mdx}",
    "./pages/**/*.{js,jsx,ts,tsx}",
  ],
  theme: {
    extend: {
      colors: {
        'robotic-black': '#0a0a0a',
        'robotic-dark': '#1a1a1a',
        'robotic-gray': '#2a2a2a',
        'robotic-cyan': '#00d4ff',
        'robotic-cyan-dark': '#00a3c2',
      },
      backgroundImage: {
        'robotic-gradient': 'linear-gradient(135deg, #0a0a0a 0%, #1a1a2e 50%, #16213e 100%)',
        'robotic-gradient-alt': 'linear-gradient(135deg, #0a0a0a 0%, #1a1a1a 50%, #0f0f0f 100%)',
      }
    },
  },
  plugins: [],
}